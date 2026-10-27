#include "BMP180_ESP32.h"
#include <Arduino.h>
#include <DHT11.h>
#include <GY271.h>
#include <GY521.h>
#include <Wire.h>
#include <math.h>

// GY271 Setup
QMC5883LCompass compass;

// DHT11 Setup
DHT11 dht11(37);

// BMP180 Setup
SFE_BMP180 bmp180;
#define SDA_PIN 36
#define SCL_PIN 35
#define P0 1013.25

// MPU6050 Setup
MPU6050 mpu;

// Telemetry variables (updated slower than the main loop)
float current_altitude = 0.0;
float current_temp = 0.0;
unsigned long last_slow_read = 0;

void setup() {
  Serial.begin(115200);
  delay(2000); // Crucial for USB CDC

  // GY271 initialization (Fixed address based on your I2C scan)
  compass.setADDR(0x2C);
  compass.init();

  // BMP180 initialization
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!bmp180.begin()) {
    Serial.println("BMP180_FAILED");
  }

  // MPU6050 initialization
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    delay(500);
  }
  mpu.setAccelPowerOnDelay(MPU6050_DELAY_3MS);
}

void loop() {
  // 1. Read fast sensors (IMU + Compass) for smooth 3D movement
  Vector normAccel = mpu.readNormalizeAccel();
  Vector normGyro = mpu.readNormalizeGyro();

  compass.read();
  int heading = compass.getAzimuth(); // Absolute heading

  // 2. Read slow sensors every 2 seconds (Barometer & DHT11)
  if (millis() - last_slow_read > 2000) {
    last_slow_read = millis();

    // DHT11
    int dht_temp = dht11.readTemperature();
    if (dht_temp != DHT11::ERROR_CHECKSUM && dht_temp != DHT11::ERROR_TIMEOUT) {
      current_temp = dht_temp;
    }

    // BMP180
    double bmpT, pressure;
    if (bmp180.startTemperature() && bmp180.getTemperature(bmpT)) {
      if (bmp180.startPressure(3) && bmp180.getPressure(pressure, bmpT)) {
        current_altitude = bmp180.altitude(pressure, P0);
      }
    }
  }

  // 3. Send strictly formatted CSV to the Python Viewer
  // Format: t_ms, ax, ay, az, gx, gy, gz, mag_heading, altitude, temp
  Serial.printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.2f,%.2f\n", millis(),
                normAccel.XAxis, normAccel.YAxis, normAccel.ZAxis,
                normGyro.XAxis, normGyro.YAxis, normGyro.ZAxis, heading,
                current_altitude, current_temp);

  // 4. 20ms delay = 50Hz update rate (perfect for smooth 3D rendering)
  delay(20);
}
