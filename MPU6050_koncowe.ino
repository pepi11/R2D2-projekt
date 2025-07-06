#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <QMC5883LCompass.h>

// Obiekty
MPU6050 mpu;
QMC5883LCompass compass;

// DMP
bool dmpReady = false;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
uint16_t packetSize;

// Filtrowany kąt
float fusedYaw = 0;
float alpha = 0.98;  // Waga MPU (żyroskopu)

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // === MPU6050 ===
  Serial.println("Inicjalizacja MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Błąd: MPU6050 nie wykryto");
    while (1);
  }

  uint8_t devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    Serial.println("Kalibracja MPU... nie ruszaj czujnika");
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
  } else {
    Serial.print("Błąd DMP (kod ");
    Serial.print(devStatus);
    Serial.println(")");
    while (1);
  }

  // === QMC5883L ===
  Serial.println("Inicjalizacja QMC5883L...");
  compass.init();
  delay(500);

  Serial.println("System gotowy.");
}

void loop() {
  if (!dmpReady) return;

  // Odczyt DMP
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float yawDeg = ypr[0] * 180.0 / M_PI;
    if (yawDeg < 0) yawDeg += 360.0;

    // Odczyt kompasu
    compass.read();
    int heading = compass.getAzimuth();
    if (heading < 0) heading += 360;

    // Filtr komplementarny
    fusedYaw = alpha * yawDeg + (1 - alpha) * heading;
    if (fusedYaw < 0) fusedYaw += 360.0;
    if (fusedYaw >= 360.0) fusedYaw -= 360.0;

    // Wyświetlenie
    Serial.print("Yaw (MPU): ");
    Serial.print(yawDeg, 2);
    Serial.print("°\tHeading (QMC): ");
    Serial.print(heading);
    Serial.print("°\tFused Yaw: ");
    Serial.print(fusedYaw, 2);
    Serial.println("°");
  }

  delay(100);
}
