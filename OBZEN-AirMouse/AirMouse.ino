#include <bluefruit.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <math.h>
#include "nrf_gpio.h"

BLEDis bledis;
BLEHidAdafruit blehid;
LSM6DS3 imu(I2C_MODE, 0x6A);

// ==========================================
// 1. AIR MOUSE GRIP & SMOOTH ARROW TUNING
// ==========================================
const bool  AUTO_ORIENTATION     = true;   // true = auto-detect horizontal vs upright; false = use MANUAL_HOLD_MODE
const int   MANUAL_HOLD_MODE     = 1;      // 1 = horizontal/remote grip, 2 = upright grip

// Horizontal (remote/flat) grip signs
const float PITCH_SIGN           = +1.0f;  // Flip between +1.0f and -1.0f if up/down is reversed
const float YAW_SIGN             = -1.0f;  // Flip between +1.0f and -1.0f if left/right is reversed

// Upright (wand/vertical) grip tuning
const int   UPRIGHT_PITCH_AXIS   = 1;      // 1 = gx, 2 = gy, 3 = gz
const int   UPRIGHT_YAW_AXIS     = 3;      // 1 = gx, 2 = gy, 3 = gz
const float UPRIGHT_PITCH_SIGN   = +1.0f;  // Flip between +1.0f and -1.0f if upright up/down is reversed
const float UPRIGHT_YAW_SIGN     = -1.0f;  // Flip between +1.0f and -1.0f if upright left/right is reversed

const float DEADZONE_X           = 3.8f;   // Horizontal deadzone (deg/s)
const float DEADZONE_Y           = 3.0f;   // Lower vertical deadzone so slow up/down moves do not stick
const float SMOOTHING            = 0.42f;  // Responsive low-pass filter (smooth without trailing lag)
const float BASE_SENSITIVITY     = 0.35f;  // Steady, natural baseline cursor speed
const float ACCEL_GAIN           = 0.011f; // Smooth speed ramp for crossing the screen easily
const float PITCH_SCALE          = 0.75f;  // Balanced vertical multiplier without stalling small moves
const float TILT_COMP_GAIN       = 0.0f;   // 0.0f isolates pure vertical pitch without roll cross-talk

const unsigned long LOOP_INTERVAL_MS = 8;    // 125 Hz HID polling rate for smooth arrow motion
const unsigned long ORIENT_HOLD_MS   = 220;  // Debounce time to stop rapid HORIZ/UPRIGHT flickering
const int   GYRO_SAMPLES             = 300;  // ~1.5s calibration at startup (keep board still)

// ==========================================
// 2. SOFT SINGLE & DOUBLE-TAP TUNING
// ==========================================
const float         SOFT_TAP_DELTA_G        = 0.165f; // Catches gentle/soft finger taps (~0.165g+)
const float         SOFT_TAP_IMPULSE_G      = 0.085f; // Sharpness check (distinguishes soft tap from smooth glide)
const float         TAP_REARM_DELTA_G       = 0.105f; // Quick re-arm for soft 2nd tap
const float         MAX_TAP_GYRO            = 125.0f; // Allows natural hand hold while ignoring wide arm swings

const unsigned long TAP_DEBOUNCE_MS         = 70;     // Minimum gap between 1st and 2nd soft tap
const unsigned long DOUBLE_TAP_WINDOW_MS    = 420;    // Comfortable window for soft double-tap to open files
const unsigned long TAP_FREEZE_CURSOR_MS    = 110;    // Brief pixel lock during tap so arrow stays on file
const unsigned long CLICK_HOLD_MS           = 45;     // Reliable left-button press duration for Windows BLE HID
const unsigned long DOUBLE_CLICK_GAP_MS     = 85;     // Crisp gap between Click 1 and Click 2

// ==========================================
// 3. AUTO-SLEEP & TELEMETRY TUNING
// ==========================================
const bool          ENABLE_SERIAL_LOG = false; // Keep false during normal use so USB serial never adds lag
const unsigned long IDLE_SLEEP_MS     = 30000; // 30s of stillness before entering low-power sleep
const float         STILL_DELTA_G     = 0.04f; // Max accel deviation from 1.0g considered still
const float         STILL_GYRO_DPS    = 6.0f;  // Max gyro magnitude (deg/s) considered still
const unsigned long SERIAL_PRINT_MS   = 500;

#if defined(PIN_LSM6DS3_INT1)
const int WAKE_PIN = PIN_LSM6DS3_INT1;
#else
const int WAKE_PIN = 6; // Fallback on XIAO nRF52840 Sense (P0.11 / INT1)
#endif

// ==========================================
// 4. RUNTIME STATE VARIABLES
// ==========================================
enum GripOrientation {
  GRIP_HORIZONTAL = 1,
  GRIP_UPRIGHT    = 2
};

float biasX = 0.0f, biasY = 0.0f, biasZ = 0.0f;
float filtPitch = 0.0f, filtYaw = 0.0f;
float remX      = 0.0f, remY    = 0.0f;
float prevDeltaG = 0.0f;

unsigned long lastLoopMs = 0;

bool leftButtonHeld             = false;
unsigned long releaseLeftAtMs   = 0;
unsigned long clickCount        = 0;

// Soft Single / Double-Tap State
int tapCount                    = 0;
unsigned long firstTapMs        = 0;
unsigned long lastTapSpikeMs    = 0;
unsigned long freezeCursorUntil = 0;
bool tapSpikeArmed              = true;
int pendingClicks               = 0;
unsigned long nextClickAtMs     = 0;

unsigned long lastActiveMs = 0;
GripOrientation currentGrip = GRIP_HORIZONTAL;
GripOrientation candidateGrip = GRIP_HORIZONTAL;
unsigned long candidateSinceMs = 0;

float selectAxis(int axisSel, float gx, float gy, float gz) {
  if (axisSel == 1) return gx;
  if (axisSel == 2) return gy;
  return gz;
}

void calibrateGyro() {
  float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
  for (int i = 0; i < GYRO_SAMPLES; i++) {
    sumX += imu.readFloatGyroX();
    sumY += imu.readFloatGyroY();
    sumZ += imu.readFloatGyroZ();
    delay(5);
  }
  biasX = sumX / (float)GYRO_SAMPLES;
  biasY = sumY / (float)GYRO_SAMPLES;
  biasZ = sumZ / (float)GYRO_SAMPLES;
}

float applyCurve(float rate) {
  float mag = fabs(rate);
  float effective = mag;
  float delta = effective * (BASE_SENSITIVITY + effective * ACCEL_GAIN);
  return (rate < 0.0f) ? -delta : delta;
}

void updateSoftTapClicks(float deltaG, float tapImpulse, float gyroRate, unsigned long now) {
  if (!Bluefruit.connected()) {
    tapCount = 0;
    pendingClicks = 0;
    if (leftButtonHeld) {
      blehid.mouseButtonRelease();
      leftButtonHeld = false;
    }
    return;
  }

  if (leftButtonHeld && (now >= releaseLeftAtMs)) {
    blehid.mouseButtonRelease();
    leftButtonHeld = false;
  }

  // Re-arm quickly once acceleration settles below TAP_REARM_DELTA_G
  if (deltaG <= TAP_REARM_DELTA_G) {
    tapSpikeArmed = true;
  }

  // Detect a soft, sharp tap impulse without false-triggering during smooth sweeps
  if (tapSpikeArmed &&
      (deltaG >= SOFT_TAP_DELTA_G) &&
      (tapImpulse >= SOFT_TAP_IMPULSE_G) &&
      (gyroRate <= MAX_TAP_GYRO) &&
      ((now - lastTapSpikeMs) >= TAP_DEBOUNCE_MS)) {
    tapSpikeArmed = false;
    lastTapSpikeMs = now;
    lastActiveMs = now;

    // Stabilize cursor on tap so 2nd tap lands on the exact same file icon
    freezeCursorUntil = now + TAP_FREEZE_CURSOR_MS;
    remX = 0.0f;
    remY = 0.0f;

    if (tapCount == 0) {
      tapCount = 1;
      firstTapMs = now;
    } else if (tapCount == 1 && ((now - firstTapMs) <= DOUBLE_TAP_WINDOW_MS)) {
      tapCount = 0;
      if (!leftButtonHeld && pendingClicks == 0) {
        pendingClicks = 2;
        nextClickAtMs = now;
        freezeCursorUntil = now + (2 * CLICK_HOLD_MS) + DOUBLE_CLICK_GAP_MS + 45;
        if (ENABLE_SERIAL_LOG) {
          Serial.println("Soft double-tap -> double-click (open file)");
        }
      }
    }
  }

  // Single soft tap fallback if no 2nd tap arrives within DOUBLE_TAP_WINDOW_MS
  if (tapCount == 1 && ((now - firstTapMs) > DOUBLE_TAP_WINDOW_MS)) {
    tapCount = 0;
    if (!leftButtonHeld && pendingClicks == 0) {
      pendingClicks = 1;
      nextClickAtMs = now;
      if (ENABLE_SERIAL_LOG) {
        Serial.println("Single soft tap -> left-click");
      }
    }
  }

  if (!leftButtonHeld && (pendingClicks > 0) && (now >= nextClickAtMs)) {
    blehid.mouseButtonPress(MOUSE_BUTTON_LEFT);
    leftButtonHeld = true;
    releaseLeftAtMs = now + CLICK_HOLD_MS;
    pendingClicks--;
    nextClickAtMs = now + CLICK_HOLD_MS + DOUBLE_CLICK_GAP_MS;
    clickCount++;
  }
}

GripOrientation rawOrientationFromAccel(float ax, float ay, float az) {
  float absX = fabs(ax);
  float absY = fabs(ay);
  float absZ = fabs(az);

  if (absZ > (absX + 0.15f) && absZ > (absY + 0.15f)) {
    return GRIP_HORIZONTAL;
  }
  return GRIP_UPRIGHT;
}

GripOrientation updateOrientation(float ax, float ay, float az, unsigned long now) {
  if (!AUTO_ORIENTATION) {
    currentGrip = (MANUAL_HOLD_MODE == 2) ? GRIP_UPRIGHT : GRIP_HORIZONTAL;
    return currentGrip;
  }

  GripOrientation detected = rawOrientationFromAccel(ax, ay, az);
  if (detected != candidateGrip) {
    candidateGrip = detected;
    candidateSinceMs = now;
  } else if ((detected != currentGrip) && ((now - candidateSinceMs) >= ORIENT_HOLD_MS)) {
    currentGrip = detected;
  }

  return currentGrip;
}

void computeGripAxes(
    GripOrientation grip,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float &outPitchRate,
    float &outYawRate) {
  if (grip == GRIP_HORIZONTAL) {
    float basePitch = gy;
    float baseYaw   = gz;

    if (TILT_COMP_GAIN > 0.0f) {
      float rollRad = atan2(ay, az);
      float cosR = cos(rollRad * TILT_COMP_GAIN);
      float sinR = sin(rollRad * TILT_COMP_GAIN);
      outPitchRate = PITCH_SIGN * ((basePitch * cosR) + (baseYaw * sinR));
      outYawRate   = YAW_SIGN   * ((-basePitch * sinR) + (baseYaw * cosR));
    } else {
      outPitchRate = PITCH_SIGN * basePitch;
      outYawRate   = YAW_SIGN   * baseYaw;
    }
  } else {
    float upPitch = selectAxis(UPRIGHT_PITCH_AXIS, gx, gy, gz);
    float upYaw   = selectAxis(UPRIGHT_YAW_AXIS,   gx, gy, gz);

    outPitchRate = UPRIGHT_PITCH_SIGN * upPitch;
    outYawRate   = UPRIGHT_YAW_SIGN   * upYaw;
  }
}

void configureImuWakeOnMotion() {
  imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x80);
  imu.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x02);
  imu.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x01);
  imu.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x20);
}

void enterLowPowerSleep() {
  if (ENABLE_SERIAL_LOG) {
    Serial.println("Idle for 30s -> entering low-power sleep (wake on motion)...");
    Serial.flush();
  }

  if (leftButtonHeld) {
    blehid.mouseButtonRelease();
    leftButtonHeld = false;
  }

  configureImuWakeOnMotion();

  uint32_t pinNum = g_ADigitalPinMap[WAKE_PIN];
  nrf_gpio_cfg_sense_input(pinNum, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);

  Bluefruit.Advertising.stop();
  delay(20);

  sd_power_system_off();

  while (1) {
    delay(1000);
  }
}

void startAdv() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_MOUSE);
  Bluefruit.Advertising.addService(blehid);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.start(0);
}

void setup() {
  if (ENABLE_SERIAL_LOG) {
    Serial.begin(115200);
  }
  pinMode(WAKE_PIN, INPUT);

  Bluefruit.begin();
  Bluefruit.Periph.setConnInterval(6, 12); // Fast 7.5-15ms BLE HID polling for lag-free cursor
  Bluefruit.setTxPower(4);
  Bluefruit.setName("OBZEN-AirMouse");

  bledis.setManufacturer("OBZEN");
  bledis.setModel("XIAO-nRF52840-AirMouse");
  bledis.begin();
  blehid.begin();

  if (imu.begin() != 0) {
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, LOW);
    while (1) delay(100);
  }

  Wire.setClock(400000); // 400 kHz Fast Mode I2C

  delay(1000);
  calibrateGyro();
  startAdv();

  lastLoopMs = millis();
  lastActiveMs = millis();
  candidateSinceMs = millis();
}

void loop() {
  unsigned long now = millis();
  if ((now - lastLoopMs) < LOOP_INTERVAL_MS) {
    return;
  }
  lastLoopMs = now;

  float gx = imu.readFloatGyroX() - biasX;
  float gy = imu.readFloatGyroY() - biasY;
  float gz = imu.readFloatGyroZ() - biasZ;

  float ax = imu.readFloatAccelX();
  float ay = imu.readFloatAccelY();
  float az = imu.readFloatAccelZ();

  float accelMag   = sqrt(ax * ax + ay * ay + az * az);
  float gyroRate   = sqrt(gx * gx + gy * gy + gz * gz);
  float deltaG     = fabs(accelMag - 1.0f);
  float tapImpulse = fabs(deltaG - prevDeltaG);
  prevDeltaG       = deltaG;

  if ((deltaG > STILL_DELTA_G) || (gyroRate > STILL_GYRO_DPS)) {
    lastActiveMs = now;
  } else if ((now - lastActiveMs) >= IDLE_SLEEP_MS) {
    enterLowPowerSleep();
  }

  updateSoftTapClicks(deltaG, tapImpulse, gyroRate, now);

  currentGrip = updateOrientation(ax, ay, az, now);

  float rawPitch = 0.0f;
  float rawYaw   = 0.0f;
  computeGripAxes(currentGrip, gx, gy, gz, ax, ay, az, rawPitch, rawYaw);

  filtPitch = (SMOOTHING * rawPitch) + ((1.0f - SMOOTHING) * filtPitch);
  filtYaw   = (SMOOTHING * rawYaw)   + ((1.0f - SMOOTHING) * filtYaw);

  bool activePitch = (fabs(filtPitch) >= DEADZONE_Y);
  bool activeYaw   = (fabs(filtYaw)   >= DEADZONE_X);

  bool cursorFrozen = (now < freezeCursorUntil) || leftButtonHeld || (pendingClicks > 0);

  if (cursorFrozen) {
    remX = 0.0f;
    remY = 0.0f;
  } else {
    float stepX = activeYaw   ? applyCurve(filtYaw - ((filtYaw > 0.0f) ? DEADZONE_X : -DEADZONE_X)) : 0.0f;
    float stepY = activePitch ? (applyCurve(filtPitch - ((filtPitch > 0.0f) ? DEADZONE_Y : -DEADZONE_Y)) * PITCH_SCALE) : 0.0f;

    if (!activeYaw)   remX = 0.0f;
    if (!activePitch) remY = 0.0f;

    remX += stepX;
    remY += stepY;

    int moveX = (int)trunc(remX);
    int moveY = (int)trunc(remY);

    remX -= (float)moveX;
    remY -= (float)moveY;

    if (Bluefruit.connected()) {
      int8_t dx = (int8_t)constrain(moveX, -127, 127);
      int8_t dy = (int8_t)constrain(moveY, -127, 127);

      if (dx != 0 || dy != 0) {
        blehid.mouseMove(dx, dy);
      }
    }
  }

  if (ENABLE_SERIAL_LOG) {
    static float windowMaxDeltaG = 0.0f;
    static float windowMaxGyro   = 0.0f;
    if (deltaG > windowMaxDeltaG) windowMaxDeltaG = deltaG;
    if (gyroRate > windowMaxGyro) windowMaxGyro = gyroRate;

    static unsigned long lastPrintTime = 0;
    if (now - lastPrintTime >= SERIAL_PRINT_MS) {
      lastPrintTime = now;
      Serial.print("grip:");
      Serial.print((currentGrip == GRIP_HORIZONTAL) ? "HORIZ" : "UPRIGHT");
      Serial.print(" | maxDeltaG:");
      Serial.print(windowMaxDeltaG, 3);
      Serial.print(" | maxGyro:");
      Serial.print(windowMaxGyro, 1);
      Serial.print(" | clicks:");
      Serial.print(clickCount);
      Serial.print(" | idleSec:");
      Serial.println((now - lastActiveMs) / 1000);
      windowMaxDeltaG = 0.0f;
      windowMaxGyro   = 0.0f;
    }
  }
}
