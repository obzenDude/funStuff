#include <ArduinoBLE.h>
#include "LSM6DS3.h"
#include "Wire.h"

LSM6DS3 myIMU(I2C_MODE, 0x6A);

// One packet = 16 bytes
struct __attribute__((packed)) Packet {
  uint32_t t;            // ms since power-on
  int16_t ax, ay, az;    // g x 1000
  int16_t gx, gy, gz;    // deg/s x 10
};
Packet pkt;

BLEService imuService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic imuChar("19B10001-E8F2-537E-4F6C-D104768A1214",
                          BLERead | BLENotify, sizeof(Packet));

unsigned long lastSend = 0;
unsigned long lastPrint = 0;
bool wasConnected = false;

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000);   // wait max 3 s, then continue

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);              // LED off (active low)

  if (myIMU.begin() != 0) {
    Serial.println("IMU failed");
    while (1);
  }
  Serial.println("IMU ok");

  if (!BLE.begin()) {
    Serial.println("BLE failed");
    while (1);
  }
  Serial.println("BLE ok");

  BLE.setLocalName("XIAO-IMU");
  BLE.setDeviceName("XIAO-IMU");
  BLE.setAdvertisedService(imuService);
  imuService.addCharacteristic(imuChar);
  BLE.addService(imuService);
  BLE.advertise();

  Serial.print("Advertising as XIAO-IMU, address: ");
  Serial.println(BLE.address());
  Serial.println("ax\tay\taz\tgx\tgy\tgz");
}

void loop() {
  BLE.poll();

  bool connected = BLE.connected();
  if (connected != wasConnected) {
    wasConnected = connected;
    digitalWrite(LED_BUILTIN, connected ? LOW : HIGH);   // LED on when connected
    Serial.println(connected ? "Connected" : "Disconnected");
    if (!connected) BLE.advertise();                     // advertise again
  }

  // Read the sensor every 20 ms (50 Hz), connected or not
  if (millis() - lastSend >= 20) {
    lastSend = millis();

    float ax = myIMU.readFloatAccelX();
    float ay = myIMU.readFloatAccelY();
    float az = myIMU.readFloatAccelZ();
    float gx = myIMU.readFloatGyroX();
    float gy = myIMU.readFloatGyroY();
    float gz = myIMU.readFloatGyroZ();

    // Send over Bluetooth only when the laptop is connected
    if (connected) {
      pkt.t  = millis();
      pkt.ax = (int16_t)(ax * 1000);
      pkt.ay = (int16_t)(ay * 1000);
      pkt.az = (int16_t)(az * 1000);
      pkt.gx = (int16_t)(gx * 10);
      pkt.gy = (int16_t)(gy * 10);
      pkt.gz = (int16_t)(gz * 10);
      imuChar.writeValue((uint8_t*)&pkt, sizeof(pkt));
    }

    // Print on the Serial Monitor 5 times per second (easy to read)
    if (Serial && millis() - lastPrint >= 200) {
      lastPrint = millis();
      Serial.print(ax); Serial.print("\t");
      Serial.print(ay); Serial.print("\t");
      Serial.print(az); Serial.print("\t");
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.println(gz);
    }
  }
}
