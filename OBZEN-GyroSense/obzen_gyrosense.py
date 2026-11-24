"""
OBZEN GyroSense — 3D Airframe Telemetry Visualizer for XIAO nRF52840 Sense.
Startup & Dual-Mode Control:
- Calibrates for 2s while standing straight UPWARD, then HOLDS in the UPWARD position
  until you choose a tracking mode:
    * [1] Fixed Mode (1)       -> Standard fixed mapping (USB = Tail, Opposite USB = Nose)
    * [2] Assign Chip Pose (2) -> Assigns whichever side of the chip is currently pointing UP (top)
                                  as the rocket's head/top reference without jumping the rocket
- Pressing Recalibrate (R) returns the rocket UPWARD, recalibrates for 2s, and waits for [1] or [2].

Keys & Buttons:
  * 1 = Fixed Mode (original USB=Tail, Nose=Opposite USB tracking)
  * 2 = Assign Chip Pose (binds current top-pointing side of chip as rocket head)
  * Z = Zero Turn (point rocket straight ahead)
  * R = Recalibrate (returns rocket UPWARD for 2s calibration and waits for mode choice)
  * F = Flip Turn L/R   |   P = Flip Pitch Up/Down   |   B = Flip Roll L/R
  * [ / ] = Adjust Turn Sensitivity   |   Esc = Exit Fullscreen   |   Q = Quit
"""
import asyncio
import math
import struct
import sys
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import PolyCollection
from matplotlib.widgets import Button

# ---------------- Settings ----------------
DEVICE_NAME = "XIAO-IMU"
CHAR_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
CALIB_SECONDS = 2.0

YAW_SIGN = -1.0 if "--flip" in sys.argv else 1.0
PITCH_SIGN = 1.0 if "--flippitch" in sys.argv else -1.0
ROLL_SIGN = -1.0 if "--fliproll" in sys.argv else 1.0
YAW_SCALE = 1.0

WIFI = "--wifi" in sys.argv
DEMO = "--demo" in sys.argv


def norm(a, b, c):
    return math.sqrt(a * a + b * b + c * c)


def _wrap_deg(angle):
    """Wrap angle smoothly into [-180, 180) so continuous 360° rotations never jump."""
    return (angle + 180.0) % 360.0 - 180.0


def _shortest_angle_diff(target, current):
    """Shortest signed angular difference (target - current) in degrees."""
    return (target - current + 180.0) % 360.0 - 180.0


# ---------------- 3x3 Rotation Matrix Helpers (for Assign Chip Pose) ----------------
def _mat_identity():
    return (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )


def _mat_mul(A, B):
    return tuple(
        tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _mat_transpose(M):
    return (
        (M[0][0], M[1][0], M[2][0]),
        (M[0][1], M[1][1], M[2][1]),
        (M[0][2], M[1][2], M[2][2]),
    )


def _euler_to_mat(roll_deg, pitch_deg, yaw_deg):
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    Rz = (
        (cr, -sr, 0.0),
        (sr, cr, 0.0),
        (0.0, 0.0, 1.0),
    )
    Rx = (
        (1.0, 0.0, 0.0),
        (0.0, cp, sp),
        (0.0, -sp, cp),
    )
    Ry = (
        (cy, 0.0, -sy),
        (0.0, 1.0, 0.0),
        (sy, 0.0, cy),
    )
    return _mat_mul(Ry, _mat_mul(Rx, Rz))


def _mat_to_euler(M):
    sp = max(-1.0, min(1.0, M[1][2]))
    pitch = math.degrees(math.asin(sp))
    cp = math.cos(math.radians(pitch))

    if abs(cp) > 1e-4:
        roll = math.degrees(math.atan2(-M[1][0], M[1][1]))
        yaw = math.degrees(math.atan2(-M[0][2], M[2][2]))
    else:
        roll = 0.0
        yaw = math.degrees(math.atan2(M[2][0], M[0][0]))

    return _wrap_deg(roll), _wrap_deg(pitch), _wrap_deg(yaw)


def _vec_to_world_up_matrix(up_body):
    """
    Given a unit vector `up_body` in the chip's body frame that is currently
    pointing toward world UP (+Y), builds a rotation matrix R that maps
    `up_body` -> world +Y, aligning the rocket's nose (+Z) with that top direction.
    """
    n = norm(*up_body) or 1.0
    ux, uy, uz = up_body[0] / n, up_body[1] / n, up_body[2] / n

    if abs(ux) < 0.8:
        rx, ry, rz = 1.0, 0.0, 0.0
    else:
        rx, ry, rz = 0.0, 1.0, 0.0

    # Right axis = ref x up
    tx = ry * uz - rz * uy
    ty = rz * ux - rx * uz
    tz = rx * uy - ry * ux
    tn = norm(tx, ty, tz) or 1.0
    tx, ty, tz = tx / tn, ty / tn, tz / tn

    # Forward/nose axis = up x right
    fx = uy * tz - uz * ty
    fy = uz * tx - ux * tz
    fz = ux * ty - uy * tx

    return (
        (tx, ux, fx),
        (ty, uy, fy),
        (tz, uz, fz),
    )


class OrientationTracker:
    def __init__(self):
        self.lock = threading.Lock()
        self.conn_status = "Starting..."
        self.reset()

    def set_status(self, status):
        with self.lock:
            self.conn_status = status

    def on_reconnect(self):
        with self.lock:
            self.last_t = None

    def reset(self):
        with self.lock:
            self.calib = []
            self.calib_start = None
            self.calibrated = False
            self.bias = [0.0, 0.0, 0.0]
            self.g = [0.0, 0.0, 1.0]
            self.last_t = None

            # Default upright launchpad pose while uncalibrated & while waiting for mode selection
            self.roll = 0.0
            self.pitch = 90.0
            self.yaw = 0.0

            # Mode: None (stopped upright waiting for user choice), "fixed", or "assigned"
            self.mode = None
            self.R_offset = _mat_identity()
            self.R_rocket = _euler_to_mat(0.0, 90.0, 0.0)

    def start_fixed_mode(self):
        """Activates the original fixed head/bottom mapping."""
        with self.lock:
            if not self.calibrated:
                return
            self.mode = "fixed"
            self.R_offset = _mat_identity()
            self.R_rocket = _euler_to_mat(self.roll, self.pitch, self.yaw)

    def assign_chip_pose(self, target_R=None):
        """
        Assigns whichever side of the physical chip is currently pointing UP (top)
        as the rocket's head/top reference, without jumping the rocket.
        """
        with self.lock:
            if not self.calibrated:
                return

            gn = norm(*self.g) or 1.0
            top_side_body = (-self.g[0] / gn, -self.g[1] / gn, -self.g[2] / gn)

            R_chip = _euler_to_mat(self.roll, self.pitch, self.yaw)
            R_top_aligned = _vec_to_world_up_matrix(top_side_body)

            if target_R is not None:
                R_target = target_R
                self.R_offset = _mat_mul(R_target, _mat_transpose(R_chip))
            else:
                self.R_offset = _mat_mul(R_top_aligned, _mat_transpose(R_chip))

            self.R_rocket = _mat_mul(self.R_offset, R_chip)
            self.mode = "assigned"

    def zero_yaw(self):
        with self.lock:
            if self.mode == "assigned":
                _, _, disp_y = _mat_to_euler(self.R_rocket)
                Ry_cancel = _euler_to_mat(0.0, 0.0, -disp_y)
                self.R_offset = _mat_mul(Ry_cancel, self.R_offset)
                R_chip = _euler_to_mat(self.roll, self.pitch, self.yaw)
                self.R_rocket = _mat_mul(self.R_offset, R_chip)
            elif self.mode == "fixed":
                self.yaw = 0.0
                self.R_rocket = _euler_to_mat(self.roll, self.pitch, 0.0)

    def _accel_angles(self, ax, ay, az):
        """
        Chip = Rocket (USB at back, nose opposite USB):
        - Flat face-up (az = +1): Pitch = 0°, Roll = 0°
        - Nose lifted UP (opposite USB up): Pitch = +90° (head points UP)
        - Nose tilted DOWN (opposite USB down): Pitch = -90° (head points DOWN)
        - Rolled onto LEFT side: Roll = -90°
        - Rolled onto RIGHT side: Roll = +90°
        """
        roll = ROLL_SIGN * math.degrees(math.atan2(ay, math.hypot(ax, az)))
        pitch = PITCH_SIGN * math.degrees(math.atan2(-ax, math.hypot(ay, az)))
        return roll, pitch

    def update(self, t_ms, ax, ay, az, gx, gy, gz):
        t = t_ms / 1000.0
        with self.lock:
            if not self.calibrated:
                # Hold rocket in upward position until calibration finishes
                self.roll = 0.0
                self.pitch = 90.0
                self.yaw = 0.0
                self.R_rocket = _euler_to_mat(0.0, 90.0, 0.0)

                if self.calib_start is None or t < self.calib_start:
                    self.calib_start = t
                    self.calib = []
                self.calib.append((ax, ay, az, gx, gy, gz))
                if t - self.calib_start >= CALIB_SECONDS and len(self.calib) >= 10:
                    n = len(self.calib)
                    m = [sum(c[i] for c in self.calib) / n for i in range(6)]
                    sd = [
                        math.sqrt(sum((c[i] - m[i]) ** 2 for c in self.calib) / n)
                        for i in range(6)
                    ]
                    if max(sd[0:3]) > 0.12 or max(sd[3:6]) > 8.0:
                        self.calib = []
                        self.calib_start = t
                        return
                    self.g = m[0:3]
                    self.bias = m[3:6]
                    self.roll, self.pitch = self._accel_angles(m[0], m[1], m[2])
                    self.yaw = 0.0
                    # Keep R_rocket UPWARD while waiting for user to choose Fixed Mode (1) or Assign Chip Pose (2)
                    self.R_rocket = _euler_to_mat(0.0, 90.0, 0.0)
                    self.calibrated = True
                    self.last_t = t
                return

            if self.last_t is None or t < self.last_t:
                self.last_t = t
                return

            dt = t - self.last_t
            self.last_t = t
            if dt <= 0 or dt > 0.5:
                return
            dt = min(dt, 0.04)

            k = 0.08
            self.g = [
                (1 - k) * self.g[0] + k * ax,
                (1 - k) * self.g[1] + k * ay,
                (1 - k) * self.g[2] + k * az,
            ]

            r = [gx - self.bias[0], gy - self.bias[1], gz - self.bias[2]]
            if norm(*r) < 3.0:
                self.bias = [b + 0.05 * ri for b, ri in zip(self.bias, r)]
                r = [0.0, 0.0, 0.0]

            acc_roll, acc_pitch = self._accel_angles(ax, ay, az)
            alpha = 0.82

            gyro_pitch = _wrap_deg(self.pitch + PITCH_SIGN * r[1] * dt)
            self.pitch = _wrap_deg(
                gyro_pitch + (1.0 - alpha) * _shortest_angle_diff(acc_pitch, gyro_pitch)
            )

            gyro_roll = _wrap_deg(self.roll + ROLL_SIGN * r[0] * dt)
            if math.hypot(ay, az) > 0.18:
                self.roll = _wrap_deg(
                    gyro_roll + (1.0 - alpha) * _shortest_angle_diff(acc_roll, gyro_roll)
                )
            else:
                self.roll = gyro_roll

            # Project gyro rotation onto world vertical (gravity) axis
            gn = norm(*self.g) or 1.0
            gx_u, gy_u, gz_u = self.g[0] / gn, self.g[1] / gn, self.g[2] / gn
            world_yaw_rate = r[0] * gx_u + r[1] * gy_u + r[2] * gz_u
            if abs(world_yaw_rate) < 1.2:
                world_yaw_rate = 0.0
            self.yaw = _wrap_deg(self.yaw + YAW_SIGN * YAW_SCALE * world_yaw_rate * dt)

            # Update rocket matrix only after a mode is selected
            if self.mode == "fixed":
                self.R_rocket = _euler_to_mat(self.roll, self.pitch, self.yaw)
            elif self.mode == "assigned":
                R_chip = _euler_to_mat(self.roll, self.pitch, self.yaw)
                self.R_rocket = _mat_mul(self.R_offset, R_chip)
            else:
                # Stopped in UPWARD position until user presses Fixed Mode (1) or Assign Chip Pose (2)
                self.R_rocket = _euler_to_mat(0.0, 90.0, 0.0)

    def snapshot(self):
        with self.lock:
            if self.mode == "assigned":
                disp_r, disp_p, disp_y = _mat_to_euler(self.R_rocket)
            elif self.mode == "fixed":
                disp_r, disp_p, disp_y = self.roll, self.pitch, self.yaw
            else:
                disp_r, disp_p, disp_y = 0.0, 90.0, 0.0
            return (
                self.calibrated,
                self.mode,
                self.R_rocket,
                disp_r,
                disp_p,
                disp_y,
                self.conn_status,
            )


# ---------------- Fast, Smooth Grom-E1 3D Mesh ----------------
def _ring(cx, cy, z, r, segs):
    return [
        (cx + r * math.cos(k * 2.0 * math.pi / segs), cy + r * math.sin(k * 2.0 * math.pi / segs), z)
        for k in range(segs)
    ]


def build_grom_e1_smooth_mesh():
    segs = 32
    faces = []

    RADOME_GREY = (0.43, 0.45, 0.48)
    HULL_WHITE = (0.94, 0.95, 0.96)
    SPINE_SIDE = (0.85, 0.87, 0.89)
    FIN_BODY = (0.88, 0.90, 0.92)
    SEAM_LINE = (0.30, 0.32, 0.35)
    BELLY_DARK = (0.24, 0.25, 0.28)
    NOZZLE_BORE = (0.12, 0.13, 0.15)

    nose_tip = (0.0, 0.0, 0.860)
    radome_stations = [
        (0.830, 0.018),
        (0.775, 0.042),
        (0.700, 0.066),
        (0.610, 0.086),
        (0.515, 0.099),
        (0.445, 0.102),
    ]
    rad_rings = [_ring(0.0, 0.0, z_s, r_s, segs) for z_s, r_s in radome_stations]
    r_collar = _ring(0.0, 0.0, 0.438, 0.102, segs)

    for k in range(segs):
        nk = (k + 1) % segs
        faces.append(([nose_tip, rad_rings[0][k], rad_rings[0][nk]], RADOME_GREY))
        for idx in range(len(rad_rings) - 1):
            r_top = rad_rings[idx]
            r_bot = rad_rings[idx + 1]
            faces.append(([r_top[k], r_bot[k], r_bot[nk], r_top[nk]], RADOME_GREY))
        faces.append(([rad_rings[-1][k], r_collar[k], r_collar[nk], rad_rings[-1][nk]], SEAM_LINE))

    z_sections = [
        (0.438, 0.345, HULL_WHITE),
        (0.345, 0.340, SEAM_LINE),
        (0.340, 0.265, HULL_WHITE),
        (0.265, 0.260, SEAM_LINE),
        (0.260, 0.065, HULL_WHITE),
        (0.065, 0.060, SEAM_LINE),
        (0.060, -0.145, HULL_WHITE),
        (-0.145, -0.150, SEAM_LINE),
        (-0.150, -0.365, HULL_WHITE),
        (-0.365, -0.370, SEAM_LINE),
        (-0.370, -0.560, HULL_WHITE),
    ]

    rad = 0.102
    for z_top, z_bot, col in z_sections:
        rt = _ring(0.0, 0.0, z_top, rad, segs)
        rb = _ring(0.0, 0.0, z_bot, rad, segs)
        for k in range(segs):
            nk = (k + 1) % segs
            if col == HULL_WHITE and z_top in (0.438, -0.150, -0.370) and k in (3, 13, 19, 29):
                c = SEAM_LINE
            else:
                c = col
            faces.append(([rt[k], rb[k], rb[nk], rt[nk]], c))

    r_aft_1 = _ring(0.0, 0.0, -0.560, 0.102, segs)
    r_aft_2 = _ring(0.0, 0.0, -0.620, 0.094, segs)
    r_aft_end = _ring(0.0, 0.0, -0.665, 0.084, segs)
    r_aft_rim = _ring(0.0, 0.0, -0.665, 0.066, segs)
    r_aft_bore = _ring(0.0, 0.0, -0.610, 0.062, segs)

    for k in range(segs):
        nk = (k + 1) % segs
        faces.append(([r_aft_1[k], r_aft_2[k], r_aft_2[nk], r_aft_1[nk]], HULL_WHITE))
        faces.append(([r_aft_2[k], r_aft_end[k], r_aft_end[nk], r_aft_2[nk]], SPINE_SIDE))
        faces.append(([r_aft_end[k], r_aft_rim[k], r_aft_rim[nk], r_aft_end[nk]], HULL_WHITE))
        faces.append(([r_aft_rim[k], r_aft_bore[k], r_aft_bore[nk], r_aft_rim[nk]], SEAM_LINE))
        faces.append(([(0.0, 0.0, -0.610), r_aft_bore[nk], r_aft_bore[k]], NOZZLE_BORE))

    sw = 0.019
    y_base = 0.097
    y_top = 0.116
    z_sp_t = 0.420
    z_sp_b = -0.545

    faces.append((
        [(-sw, y_base, z_sp_t + 0.035), (sw, y_base, z_sp_t + 0.035), (sw * 0.80, y_top, z_sp_t), (-sw * 0.80, y_top, z_sp_t)],
        SPINE_SIDE,
    ))
    faces.append((
        [(-sw * 0.80, y_top, z_sp_t), (sw * 0.80, y_top, z_sp_t), (sw * 0.80, y_top, z_sp_b), (-sw * 0.80, y_top, z_sp_b)],
        HULL_WHITE,
    ))
    faces.append((
        [(-sw, y_base, z_sp_t), (-sw * 0.80, y_top, z_sp_t), (-sw * 0.80, y_top, z_sp_b), (-sw, y_base, z_sp_b)],
        SPINE_SIDE,
    ))
    faces.append((
        [(sw * 0.80, y_top, z_sp_t), (sw, y_base, z_sp_t), (sw, y_base, z_sp_b), (sw * 0.80, y_top, z_sp_b)],
        SPINE_SIDE,
    ))
    faces.append((
        [(-sw * 0.80, y_top, z_sp_b), (sw * 0.80, y_top, z_sp_b), (sw, y_base, z_sp_b - 0.025), (-sw, y_base, z_sp_b - 0.025)],
        SEAM_LINE,
    ))

    for z_lug in (0.17, -0.06, -0.47):
        faces.append((
            [(-0.006, y_top, z_lug + 0.011), (0.006, y_top, z_lug + 0.011), (0.006, y_top + 0.012, z_lug + 0.008), (-0.006, y_top + 0.012, z_lug + 0.008)],
            SEAM_LINE,
        ))
        faces.append((
            [(-0.011, y_top + 0.012, z_lug + 0.012), (0.011, y_top + 0.012, z_lug + 0.012), (0.011, y_top + 0.015, z_lug - 0.012), (-0.011, y_top + 0.015, z_lug - 0.012)],
            BELLY_DARK,
        ))

    kw = 0.036
    yk_base = -0.095
    yk_bot = -0.123
    zk_front = 0.415
    zk_mid = 0.075
    zk_rear = -0.380

    faces.append((
        [(-kw * 0.45, yk_base, zk_front), (kw * 0.45, yk_base, zk_front), (kw * 0.86, yk_bot, zk_mid), (-kw * 0.86, yk_bot, zk_mid)],
        HULL_WHITE,
    ))
    faces.append((
        [(-kw * 0.65, yk_base, zk_front), (-kw * 0.86, yk_bot, zk_mid), (-kw, yk_base, zk_mid)],
        SPINE_SIDE,
    ))
    faces.append((
        [(kw * 0.65, yk_base, zk_front), (kw, yk_base, zk_mid), (kw * 0.86, yk_bot, zk_mid)],
        SPINE_SIDE,
    ))

    for z_dot in (0.215, 0.188, 0.161, 0.134, 0.107):
        t_frac = (zk_front - z_dot) / (zk_front - zk_mid)
        y_dot = yk_base + t_frac * (yk_bot - yk_base) - 0.0015
        for x_off in (-0.019, 0.019):
            faces.append((
                [
                    (x_off - 0.0025, y_dot, z_dot + 0.0025),
                    (x_off + 0.0025, y_dot, z_dot + 0.0025),
                    (x_off + 0.0025, y_dot, z_dot - 0.0025),
                    (x_off - 0.0025, y_dot, z_dot - 0.0025),
                ],
                SEAM_LINE,
            ))

    faces.append((
        [(-kw * 0.86, yk_bot, zk_mid), (kw * 0.86, yk_bot, zk_mid), (kw * 0.86, yk_bot, zk_rear), (-kw * 0.86, yk_bot, zk_rear)],
        BELLY_DARK,
    ))
    faces.append((
        [(-kw, yk_base, zk_mid), (-kw * 0.86, yk_bot, zk_mid), (-kw * 0.86, yk_bot, zk_rear), (-kw, yk_base, zk_rear)],
        SEAM_LINE,
    ))
    faces.append((
        [(kw * 0.86, yk_bot, zk_mid), (kw, yk_base, zk_mid), (kw, yk_base, zk_rear), (kw * 0.86, yk_bot, zk_rear)],
        SEAM_LINE,
    ))
    faces.append((
        [(-kw * 0.86, yk_bot, zk_rear), (kw * 0.86, yk_bot, zk_rear), (kw, yk_base, zk_rear - 0.030), (-kw, yk_base, zk_rear - 0.030)],
        NOZZLE_BORE,
    ))

    for ang in (math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0):
        ca, sa = math.cos(ang), math.sin(ang)
        tx, ty = -sa, ca
        t_half = 0.0042

        r_lead = (0.100 * ca, 0.100 * sa, -0.455)
        r_trail = (0.094 * ca, 0.094 * sa, -0.615)
        tip_lead = (0.182 * ca, 0.182 * sa, -0.535)
        tip_trail = (0.182 * ca, 0.182 * sa, -0.625)

        for side in (-1.0, 1.0):
            ox, oy = side * t_half * tx, side * t_half * ty
            rl = (r_lead[0] + ox, r_lead[1] + oy, r_lead[2])
            rt = (r_trail[0] + ox, r_trail[1] + oy, r_trail[2])
            tl = (tip_lead[0] + ox, tip_lead[1] + oy, tip_lead[2])
            tt = (tip_trail[0] + ox, tip_trail[1] + oy, tip_trail[2])
            faces.append(([rl, tl, tt, rt], FIN_BODY if side > 0 else SPINE_SIDE))

        faces.append((
            [
                (tip_lead[0] - t_half * tx, tip_lead[1] - t_half * ty, tip_lead[2]),
                (tip_lead[0] + t_half * tx, tip_lead[1] + t_half * ty, tip_lead[2]),
                (tip_trail[0] + t_half * tx, tip_trail[1] + t_half * ty, tip_trail[2]),
                (tip_trail[0] - t_half * tx, tip_trail[1] - t_half * ty, tip_trail[2]),
            ],
            SEAM_LINE,
        ))
        faces.append((
            [
                (r_lead[0] - t_half * tx, r_lead[1] - t_half * ty, r_lead[2]),
                (r_lead[0] + t_half * tx, r_lead[1] + t_half * ty, r_lead[2]),
                (tip_lead[0] + t_half * tx, tip_lead[1] + t_half * ty, tip_lead[2]),
                (tip_lead[0] - t_half * tx, tip_lead[1] - t_half * ty, tip_lead[2]),
            ],
            SEAM_LINE,
        ))

    return faces


class GromE1FullScreenView:
    def __init__(self, ax):
        self.ax = ax
        self.ax.set_facecolor("#f8fafc")
        self.ax.set_xlim(0, 1)
        self.ax.set_ylim(0, 1)
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_aspect("auto")
        for spine in self.ax.spines.values():
            spine.set_visible(False)

        for gx in (0.16, 0.33, 0.50, 0.67, 0.84):
            self.ax.plot([gx, gx], [0.0, 1.0], color="#eef2f6", lw=0.7, ls=":", zorder=0)
        for gy in (0.22, 0.37, 0.52, 0.67, 0.82):
            self.ax.plot([0.0, 1.0], [gy, gy], color="#eef2f6", lw=0.7, ls=":", zorder=0)
        self.ax.plot([0.04, 0.96], [0.52, 0.52], color="#e2e8f0", lw=0.9, ls="--", zorder=1)
        self.ax.plot([0.50, 0.50], [0.12, 0.92], color="#e2e8f0", lw=0.9, ls="--", zorder=1)

        self.mesh = build_grom_e1_smooth_mesh()
        self.collection = PolyCollection([], closed=True, linewidths=0.0, antialiased=True, zorder=5)
        self.collection.set_animated(True)
        self.ax.add_collection(self.collection)

        self.R_curr = _euler_to_mat(0.0, 90.0, 0.0)
        self.roll = 0.0
        self.pitch = 90.0
        self.yaw = 0.0
        self._last_drawn = None
        self.render(force=True)

    def _screen_aspect(self):
        fig = self.ax.figure
        if fig is None:
            return 16.0 / 9.0
        w, h = fig.get_size_inches()
        return max(0.5, w / max(0.1, h))

    def snap_upright(self):
        self.R_curr = _euler_to_mat(0.0, 90.0, 0.0)
        self.roll = 0.0
        self.pitch = 90.0
        self.yaw = 0.0
        self.render()

    def update(self, target_roll, target_pitch, target_yaw):
        smooth = 0.78
        self.roll = _wrap_deg(self.roll + smooth * _shortest_angle_diff(target_roll, self.roll))
        self.pitch = _wrap_deg(self.pitch + smooth * _shortest_angle_diff(target_pitch, self.pitch))
        self.yaw = _wrap_deg(self.yaw + smooth * _shortest_angle_diff(target_yaw, self.yaw))
        self.R_curr = _euler_to_mat(self.roll, self.pitch, self.yaw)
        self.render()

    def update_matrix(self, R_target, disp_roll, disp_pitch, disp_yaw):
        smooth = 0.78
        self.R_curr = tuple(
            tuple(
                self.R_curr[i][j] + smooth * (R_target[i][j] - self.R_curr[i][j])
                for j in range(3)
            )
            for i in range(3)
        )
        r0 = self.R_curr[0]
        n0 = norm(*r0) or 1.0
        r0 = (r0[0] / n0, r0[1] / n0, r0[2] / n0)

        r1 = self.R_curr[1]
        d01 = r0[0] * r1[0] + r0[1] * r1[1] + r0[2] * r1[2]
        r1 = (r1[0] - d01 * r0[0], r1[1] - d01 * r0[1], r1[2] - d01 * r0[2])
        n1 = norm(*r1) or 1.0
        r1 = (r1[0] / n1, r1[1] / n1, r1[2] / n1)

        r2 = (
            r0[1] * r1[2] - r0[2] * r1[1],
            r0[2] * r1[0] - r0[0] * r1[2],
            r0[0] * r1[1] - r0[1] * r1[0],
        )
        self.R_curr = (r0, r1, r2)
        self.roll = disp_roll
        self.pitch = disp_pitch
        self.yaw = disp_yaw
        self.render()

    def render(self, force=False):
        aspect = round(self._screen_aspect(), 3)
        state = (
            round(self.R_curr[0][2], 4),
            round(self.R_curr[1][2], 4),
            round(self.R_curr[2][2], 4),
            round(self.R_curr[0][1], 4),
            aspect,
        )
        if not force and state == self._last_drawn:
            return
        self._last_drawn = state

        M = self.R_curr
        cam_pitch = math.radians(14.0)
        ccp, scp = math.cos(cam_pitch), math.sin(cam_pitch)

        lx, ly, lz = -0.35, 0.85, -0.39
        items = []

        scale_y = 1.65
        scale_x = scale_y / aspect

        for verts, rgb in self.mesh:
            t_verts = []
            for vx, vy, vz in verts:
                vz_c = vz - 0.08

                xw = M[0][0] * vx + M[0][1] * vy + M[0][2] * vz_c
                yw = M[1][0] * vx + M[1][1] * vy + M[1][2] * vz_c
                zw = M[2][0] * vx + M[2][1] * vy + M[2][2] * vz_c

                xc = xw
                yc = yw * ccp - zw * scp
                zc = yw * scp + zw * ccp
                t_verts.append((xc, yc, zc))

            ux, uy, uz = (
                t_verts[1][0] - t_verts[0][0],
                t_verts[1][1] - t_verts[0][1],
                t_verts[1][2] - t_verts[0][2],
            )
            wx, wy, wz = (
                t_verts[2][0] - t_verts[0][0],
                t_verts[2][1] - t_verts[0][1],
                t_verts[2][2] - t_verts[0][2],
            )
            nx, ny, nz = uy * wz - uz * wy, uz * wx - ux * wz, ux * wy - uy * wx
            nn = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            nx, ny, nz = nx / nn, ny / nn, nz / nn

            ndotl = nx * lx + ny * ly + nz * lz
            wrap = 0.5 * (ndotl + 1.0)
            shade = 0.78 + 0.22 * wrap
            face_col = (
                min(1.0, rgb[0] * shade),
                min(1.0, rgb[1] * shade),
                min(1.0, rgb[2] * shade),
            )

            avg_depth = sum(v[2] for v in t_verts) / len(t_verts)
            proj_2d = [
                (0.50 + (xc * scale_x) / max(1.05, 2.38 + zc), 0.52 + (yc * scale_y) / max(1.05, 2.38 + zc))
                for xc, yc, zc in t_verts
            ]
            items.append((avg_depth, proj_2d, face_col))

        items.sort(key=lambda it: it[0], reverse=True)
        self.collection.set_verts([it[1] for it in items])
        colors = [it[2] for it in items]
        self.collection.set_facecolor(colors)
        self.collection.set_edgecolor(colors)


# ---------------- Data Sources (BLE / Wi-Fi / Demo) ----------------
def run_demo(tracker):
    tracker.set_status("Demo Mode")
    t = 0.0
    dt = 0.02
    while True:
        cycle = t % 12.0
        if cycle < 2.5:
            ax, ay, az, gx, gy, gz = 0.0, 0.0, 1.0, 0.0, 0.0, 0.0
        elif cycle < 5.0:
            ax, ay, az, gx, gy, gz = 0.0, 0.0, 1.0, 0.0, 0.0, 36.0
        elif cycle < 7.5:
            ax, ay, az, gx, gy, gz = 0.0, 0.0, 1.0, 0.0, 0.0, -36.0
        elif cycle < 10.0:
            ax, ay, az, gx, gy, gz = 0.0, 0.0, 1.0, 0.0, 0.0, -36.0
        else:
            ax, ay, az, gx, gy, gz = 0.0, 0.0, 1.0, 0.0, 0.0, 36.0
        tracker.update(int(t * 1000), ax, ay, az, gx, gy, gz)
        t += dt
        time.sleep(dt)


def run_ble(tracker):
    from bleak import BleakScanner, BleakClient

    def on_data(_, data):
        t, ax, ay, az, gx, gy, gz = struct.unpack("<Ihhhhhh", data)
        tracker.update(
            t, ax / 1000, ay / 1000, az / 1000, gx / 10, gy / 10, gz / 10
        )

    async def main():
        first_connect = True
        while True:
            msg = "Scanning for XIAO-IMU..." if first_connect else "Disconnected - Reconnecting..."
            print(msg)
            tracker.set_status(msg)
            try:
                dev = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=5.0)
                if dev is None:
                    await asyncio.sleep(0.5)
                    continue

                async with BleakClient(dev) as client:
                    if not first_connect:
                        tracker.on_reconnect()
                    first_connect = False
                    print("Connected to", DEVICE_NAME)
                    tracker.set_status("BLE: Connected")
                    await client.start_notify(CHAR_UUID, on_data)
                    while client.is_connected:
                        await asyncio.sleep(0.3)
            except Exception as exc:
                print("BLE error, retrying:", exc)
                tracker.set_status("Disconnected - Reconnecting...")
                await asyncio.sleep(1.0)

    asyncio.run(main())


def run_wifi(tracker):
    import socket

    port = 4210
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    tracker.set_status(f"Wi-Fi UDP :{port}")
    while True:
        data, _ = sock.recvfrom(64)
        if len(data) == 16:
            t, ax, ay, az, gx, gy, gz = struct.unpack("<Ihhhhhh", data)
            tracker.update(
                t, ax / 1000, ay / 1000, az / 1000, gx / 10, gy / 10, gz / 10
            )


class FullScreenController:
    def __init__(self, fig):
        self.fig = fig
        self.is_fullscreen = False

    def enter(self):
        mgr = self.fig.canvas.manager
        if mgr is None:
            return
        if not self.is_fullscreen:
            try:
                mgr.full_screen_toggle()
                self.is_fullscreen = True
            except Exception:
                pass
        win = getattr(mgr, "window", None)
        if win is not None:
            for fn_name in ("showMaximized", "state"):
                fn = getattr(win, fn_name, None)
                if callable(fn):
                    try:
                        if fn_name == "state":
                            fn("zoomed")
                        else:
                            fn()
                        break
                    except Exception:
                        pass

    def exit(self):
        mgr = self.fig.canvas.manager
        if mgr is None:
            return
        if self.is_fullscreen:
            try:
                mgr.full_screen_toggle()
                self.is_fullscreen = False
            except Exception:
                pass
        win = getattr(mgr, "window", None)
        if win is not None:
            for fn_name in ("showNormal", "state"):
                fn = getattr(win, fn_name, None)
                if callable(fn):
                    try:
                        if fn_name == "state":
                            fn("normal")
                        else:
                            fn()
                        break
                    except Exception:
                        pass

    def toggle(self):
        if self.is_fullscreen:
            self.exit()
        else:
            self.enter()


# ---------------- Main UI ----------------
def main():
    tracker = OrientationTracker()
    threading.Thread(
        target=run_demo if DEMO else run_wifi if WIFI else run_ble,
        args=(tracker,),
        daemon=True,
    ).start()

    fig = plt.figure(figsize=(16, 9), facecolor="#f8fafc")
    if fig.canvas.manager is not None:
        fig.canvas.manager.set_window_title("OBZEN GyroSense")

    fs_ctrl = FullScreenController(fig)

    ax = fig.add_axes([0.0, 0.0, 1.0, 1.0])
    rocket_view = GromE1FullScreenView(ax)

    # Top-left brand label
    ax.text(
        0.025,
        0.975,
        "OBZEN GyroSense",
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=13,
        fontweight="bold",
        color="#0f172a",
        zorder=20,
    )

    # Top-center title
    ax.text(
        0.5,
        0.975,
        "OBZEN GyroSense",
        transform=ax.transAxes,
        ha="center",
        va="top",
        fontsize=13,
        fontweight="bold",
        color="#0f172a",
        zorder=20,
    )
    status_txt = ax.text(
        0.5,
        0.942,
        "Starting...",
        transform=ax.transAxes,
        ha="center",
        va="top",
        fontsize=9.5,
        fontweight="bold",
        color="#0284c7",
        zorder=20,
    )
    status_txt.set_animated(True)

    angles_txt = ax.text(
        0.5,
        0.072,
        "Pitch (Up/Down): +90°   |   Turn (L/R): +0°   |   Roll: +0°",
        transform=ax.transAxes,
        ha="center",
        va="bottom",
        fontsize=10.5,
        fontweight="bold",
        color="#0f172a",
        bbox=dict(fc="#ffffff", ec="#cbd5e1", boxstyle="round,pad=0.36"),
        zorder=20,
    )
    angles_txt.set_animated(True)

    ax_fixed = fig.add_axes([0.03, 0.016, 0.145, 0.042])
    ax_assign = fig.add_axes([0.185, 0.016, 0.185, 0.042])
    ax_zero = fig.add_axes([0.38, 0.016, 0.135, 0.042])
    ax_recal = fig.add_axes([0.525, 0.016, 0.145, 0.042])
    ax_exit_fs = fig.add_axes([0.68, 0.016, 0.165, 0.042])
    ax_quit = fig.add_axes([0.855, 0.016, 0.115, 0.042])

    btn_fixed = Button(ax_fixed, "Fixed Mode (1)", color="#dbeafe", hovercolor="#bfdbfe")
    btn_assign = Button(ax_assign, "Assign Chip Pose (2)", color="#dcfce7", hovercolor="#bbf7d0")
    btn_zero = Button(ax_zero, "Zero Turn (Z)", color="#e2e8f0", hovercolor="#cbd5e1")
    btn_recal = Button(ax_recal, "Recalibrate (R)", color="#e2e8f0", hovercolor="#cbd5e1")
    btn_exit_fs = Button(ax_exit_fs, "Exit Fullscreen (Esc)", color="#e2e8f0", hovercolor="#cbd5e1")
    btn_quit = Button(ax_quit, "Close App (Q)", color="#fee2e2", hovercolor="#fecaca")

    for btn, col in (
        (btn_fixed, "#1d4ed8"),
        (btn_assign, "#166534"),
        (btn_zero, "#0f172a"),
        (btn_recal, "#0f172a"),
        (btn_exit_fs, "#0f172a"),
        (btn_quit, "#991b1b"),
    ):
        btn.label.set_color(col)
        btn.label.set_fontweight("bold")

    def do_fixed_mode(_=None):
        tracker.start_fixed_mode()

    def do_assign_pose(_=None):
        tracker.assign_chip_pose(target_R=rocket_view.R_curr)

    btn_fixed.on_clicked(do_fixed_mode)
    btn_assign.on_clicked(do_assign_pose)
    btn_zero.on_clicked(lambda _: tracker.zero_yaw())
    btn_recal.on_clicked(lambda _: tracker.reset())
    btn_exit_fs.on_clicked(lambda _: fs_ctrl.exit())
    btn_quit.on_clicked(lambda _: plt.close(fig))

    def update(_):
        ok, mode, R_rocket, roll, pitch, yaw, status = tracker.snapshot()
        if not ok:
            status_txt.set_text(f"{status}  |  Rocket Upright — Keep chip still for 2s to calibrate...")
            status_txt.set_color("#b45309")
            rocket_view.snap_upright()
        elif mode is None:
            status_txt.set_text(
                f"{status}  |  Calibrated & Holding Upright — Click 'Fixed Mode (1)' or 'Assign Chip Pose (2)'"
            )
            status_txt.set_color("#b45309")
            rocket_view.snap_upright()
        elif mode == "fixed":
            status_txt.set_text(
                f"{status}  |  Turn Scale: {YAW_SCALE:.2f}x ([ / ])"
            )
            status_txt.set_color("#1d4ed8")
            rocket_view.update(roll, pitch, yaw)
        else:
            status_txt.set_text(
                f"{status}  |  Turn Scale: {YAW_SCALE:.2f}x ([ / ])"
            )
            status_txt.set_color("#166534")
            rocket_view.update_matrix(R_rocket, roll, pitch, yaw)

        angles_txt.set_text(
            f"Pitch (Up/Down): {rocket_view.pitch:+04.0f}°   |   "
            f"Turn (L/R): {rocket_view.yaw:+04.0f}°   |   "
            f"Roll: {rocket_view.roll:+04.0f}°"
        )
        return [rocket_view.collection, status_txt, angles_txt]

    def on_key(event):
        global YAW_SIGN, PITCH_SIGN, ROLL_SIGN, YAW_SCALE
        if event.key == "1":
            do_fixed_mode()
        elif event.key == "2":
            do_assign_pose()
        elif event.key == "r":
            tracker.reset()
        elif event.key == "z":
            tracker.zero_yaw()
        elif event.key == "f":
            YAW_SIGN = -YAW_SIGN
        elif event.key == "p":
            PITCH_SIGN = -PITCH_SIGN
        elif event.key == "b":
            ROLL_SIGN = -ROLL_SIGN
        elif event.key == "escape":
            fs_ctrl.exit()
        elif event.key == "f11":
            fs_ctrl.toggle()
        elif event.key == "q":
            plt.close(fig)
        elif event.key == "[":
            YAW_SCALE = max(0.05, round(YAW_SCALE - 0.05, 2))
        elif event.key == "]":
            YAW_SCALE = min(1.50, round(YAW_SCALE + 0.05, 2))

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("resize_event", lambda _: rocket_view.render(force=True))
    anim = FuncAnimation(fig, update, interval=16, blit=True, cache_frame_data=False)
    fs_ctrl.enter()
    plt.show()


if __name__ == "__main__":
    main()
