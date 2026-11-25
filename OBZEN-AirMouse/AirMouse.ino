#include <bluefruit.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <math.h>
#include "nrf_gpio.h"

BLEDis bledis;
BLEHidAdafruit blehid;
LSM6DS3 imu(I2C_MODE, 0x6A);

// ==========================================
// 1. AIR MOUSE GRIP & DIRECTION TUNING
// ==========================================
const bool  AUTO_ORIENTATION     = true;   // true = auto-detect horizontal vs upright; false = use MANUAL_HOLD_MODE
const int   MANUAL_HOLD_MODE     = 1;      // 1 = horizontal/remote grip, 2 = upright grip

// Horizontal (remote/flat) grip signs
const float PITCH_SIGN           = +1.0f;  // Flip between +1.0f and -1.0f if up/down is reversed
const float YAW_SIGN             = -1.0f;  // Flip between +1.0f and -1.0f if left/right is reversed

// Upright (wand/vertical) grip tuning
// Adjust UPRIGHT_PITCH_AXIS / UPRIGHT_YAW_AXIS (1=gx, 2=gy, 3=gz) and signs for your exact upright hold
const int   UPRIGHT_PITCH_AXIS   = 1;      // 1 = gx, 2 = gy, 3 = gz
const int   UPRIGHT_YAW_AXIS     = 3;      // 1 = gx, 2 = gy, 3 = gz
const float UPRIGHT_PITCH_SIGN   = +1.0f;  // Flip between +1.0f and -1.0f if upright up/down is reversed
const float UPRIGHT_YAW_SIGN     = -1.0f;  // Flip between +1.0f and -1.0f if upright left/right is reversed

const float DEADZONE             = 7.0f;   // Lowered slightly from 8.5f for quicker start while guarding drift
const float SMOOTHING            = 0.40f;  // Balanced responsiveness and smoothness (0..1)
const float BASE_SENSITIVITY     = 0.32f;  // Increased for higher baseline cursor sensitivity
const float ACCEL_GAIN           = 0.010f; // Increased for faster movement on larger sweeps
const float PITCH_SCALE          = 1.10f;  // Slightly boosted vertical travel
const float TILT_COMP_GAIN       = 0.75f;  // Moderated roll compensation to reduce yaw-to-pitch bleed

const unsigned long ORIENT_HOLD_MS = 180;    // Debounce time to stop rapid HORIZ/UPRIGHT flickering
const int   GYRO_SAMPLES         = 400;    // ~2.0s calibration at startup (keep board still)

// ==========================================
// 2. AUTO-SLEEP & TELEMETRY TUNING
// ==========================================
const unsigned long IDLE_SLEEP_MS    = 30000; // 30s of stillness before entering low-power sleep
const float         STILL_DELTA_G    = 0.05f;  // Max accel deviation from 1.0g considered still
const float         STILL_GYRO_DPS   = 7.5f;   // Stillness threshold aligned above lower deadzone
const unsigned long SERIAL_PRINT_MS  = 500;   // Serial Monitor print interval (every 0.5s)

#if defined(PIN_LSM6DS3_INT1)
const int WAKE_PIN = PIN_LSM6DS3_INT1;
#else
const int WAKE_PIN = 6; // Fallback on XIAO nRF52840 Sense (P0.11 / INT1)
#endif

// ==========================================
// 3. RUNTIME STATE VARIABLES
// ==========================================
enum GripOrientation {
  GRIP_HORIZONTAL = 1,
  GRIP_UPRIGHT    = 2
};

float biasX = 0.0f, biasY = 0.0f, biasZ = 0.0f;
float filtPitch = 0.0f, filtYaw = 0.0f;
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
  if (mag < DEADZONE) return 0.0f;
  float effective = mag - DEADZONE;
  float delta = effective * (BASE_SENSITIVITY + effective * ACCEL_GAIN);
  return (rate < 0.0f) ? -delta : delta;
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
    float rollRad = atan2(ay, az);
    float cosR = cos(rollRad * TILT_COMP_GAIN);
    float sinR = sin(rollRad * TILT_COMP_GAIN);

    float basePitch = gy;
    float baseYaw   = gz;

    float leveledPitch = (basePitch * cosR) + (baseYaw * sinR);
    float leveledYaw   = (-basePitch * sinR) + (baseYaw * cosR);

    outPitchRate = PITCH_SIGN * leveledPitch;
    outYawRate   = YAW_SIGN   * leveledYaw;
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
  Serial.println("Idle for 30s -> entering low-power sleep (wake on motion)...");
  Serial.flush();

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
  Serial.begin(115200);
  pinMode(WAKE_PIN, INPUT);

  Bluefruit.begin();
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

  delay(1500);
  calibrateGyro();
  startAdv();

  lastActiveMs = millis();
  candidateSinceMs = millis();
}

void loop() {
  unsigned long now = millis();

  float gx = imu.readFloatGyroX() - biasX;
  float gy = imu.readFloatGyroY() - biasY;
  float gz = imu.readFloatGyroZ() - biasZ;

  float ax = imu.readFloatAccelX();
  float ay = imu.readFloatAccelY();
  float az = imu.readFloatAccelZ();

  float accelMag = sqrt(ax * ax + ay * ay + az * az);
  float gyroRate = sqrt(gx * gx + gy * gy + gz * gz);
  float deltaG   = fabs(accelMag - 1.0f);

  if ((deltaG > STILL_DELTA_G) || (gyroRate > STILL_GYRO_DPS)) {
    lastActiveMs = now;
  } else if ((now - lastActiveMs) >= IDLE_SLEEP_MS) {
    enterLowPowerSleep();
  }

  currentGrip = updateOrientation(ax, ay, az, now);

  float rawPitch = 0.0f;
  float rawYaw   = 0.0f;
  computeGripAxes(currentGrip, gx, gy, gz, ax, ay, az, rawPitch, rawYaw);

  filtPitch = (SMOOTHING * rawPitch) + ((1.0f - SMOOTHING) * filtPitch);
  filtYaw   = (SMOOTHING * rawYaw)   + ((1.0f - SMOOTHING) * filtYaw);

  if (fabs(filtPitch) < DEADZONE) filtPitch = 0.0f;
  if (fabs(filtYaw)   < DEADZONE) filtYaw   = 0.0f;

  if (Bluefruit.connected()) {
    int8_t dx = constrain((int)round(applyCurve(filtYaw)),                -127, 127);
    int8_t dy = constrain((int)round(applyCurve(filtPitch) * PITCH_SCALE), -127, 127);

    if (dx != 0 || dy != 0) {
      blehid.mouseMove(dx, dy);
    }
  }

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
    Serial.print(" | idleSec:");
    Serial.println((now - lastActiveMs) / 1000);
    windowMaxDeltaG = 0.0f;
    windowMaxGyro   = 0.0f;
  }

  delay(5);
}
