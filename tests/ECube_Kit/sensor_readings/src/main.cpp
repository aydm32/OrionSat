#include <Arduino.h>
#include <GY271.h>
#include <math.h> // For trigonometric functions
#include <DHT11.h>
#include "BMP180_ESP32.h"
#include <Wire.h>
#include <GY521.h>

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
unsigned long previousTime = 0;
float timeStep = 0.01;
float pitch = 0, roll = 0, yaw = 0;
int pitchRounds = 0, rollRounds = 0, yawRounds = 0;

void setup() {
    // Serial setup
    Serial.begin(115200);

    // GY271 initialization
    Serial.println("Initializing GY271...");
    compass.init();

    // DHT11 initialization
    Serial.println("Initializing DHT11...");
    // No explicit init function for DHT11 as it's directly accessed

    // BMP180 initialization
    Serial.println("Initializing BMP180...");
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!bmp180.begin()) {
        Serial.println("Failed to initialize BMP180 sensor");
        while (1);
    }
    Serial.println("BMP180 initialized successfully.");

    // MPU6050 initialization
    Serial.println("Initializing MPU6050...");
    while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
        Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        delay(500);
    }
    mpu.setAccelPowerOnDelay(MPU6050_DELAY_3MS);
    mpu.setThreshold(3);
    previousTime = millis();
    Serial.println("MPU6050 initialized successfully.");
}

void loop() {
    // GY271 Reading
    int x, y, z;
    float angleX, angleY, angleZ;

    compass.read();
    x = compass.getX();
    y = compass.getY();
    z = compass.getZ();

    angleX = atan2(y, z) * 180.0 / PI;
    angleY = atan2(z, x) * 180.0 / PI;
    angleZ = atan2(x, y) * 180.0 / PI;

  /*  if (angleX > 180) angleX -= 360;
    if (angleY > 180) angleY -= 360;
    if (angleZ > 180) angleZ -= 360;
*/
    Serial.print("GY271_X-axis_Angle:");
    Serial.print(angleX + 81.50);
    Serial.print(",");

    Serial.print("GY271_Y-axis_Angle:");
    Serial.print(angleY - 168.9);
    Serial.print(",");

    Serial.print("GY271_Z-axis_Angle:");
    Serial.print(angleZ + 142.45);
    Serial.print(",");

    // DHT11 Reading
    int humidity = dht11.readHumidity();
    double dhtTemperature = dht11.readTemperature();
    if (humidity != DHT11::ERROR_CHECKSUM && humidity != DHT11::ERROR_TIMEOUT) {
        Serial.print("DHT11_Humidity:");
        Serial.print(humidity);
        Serial.print(",");
    } else {
        Serial.println("DHT11 Error reading humidity");
    }
    if (dhtTemperature != DHT11::ERROR_CHECKSUM && dhtTemperature != DHT11::ERROR_TIMEOUT) {
        Serial.print("DHT11_Temperature:");
        Serial.print(dhtTemperature);
        Serial.print(",");
    } else {
        Serial.println("DHT11 Error reading temperature");
    }

    // BMP180 Reading
    double bmpTemperature, pressure, altitude;
    if (bmp180.startTemperature()) {
        delay(5);
        if (bmp180.getTemperature(bmpTemperature)) {
            Serial.print("BMP180_Temperature:");
            Serial.print(bmpTemperature);
            Serial.print(",");
        } else {
            Serial.println("BMP180 Error reading temperature");
        }
    }
    if (bmp180.startPressure(3)) {
        delay(26);
        if (bmp180.getPressure(pressure, bmpTemperature)) {
            Serial.print("BMP180_Pressure:");
            Serial.print(pressure);
            Serial.print(",");
        } else {
            Serial.println("BMP180 Error reading pressure");
        }
    }
    altitude = bmp180.altitude(pressure, P0);
    if (altitude != 0) {
        Serial.print("BMP180_Altitude:");
        Serial.print(altitude);
        Serial.print(",");
    } else {
        Serial.println("BMP180 Error reading altitude");
    }

    // MPU6050 Reading
    unsigned long currentTime = millis();
    timeStep = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    Vector normAccel = mpu.readNormalizeAccel();

    Serial.print("X_Accelerometer:");
    Serial.print(normAccel.XAxis);
    Serial.print(",");
    Serial.print("Y_Accelerometer:");
    Serial.print(normAccel.YAxis);
    Serial.print(",");
    Serial.print("Z_Accelerometer:");
    Serial.print(normAccel.ZAxis);
       Serial.println(",");


    Activites act = mpu.readActivites();
    if (act.isFreeFall) {
        Serial.println("MPU6050 Freefall Detected");
    }
    if (act.isActivity) {
        Serial.println("MPU6050 Motion Detected");
    }

    delay(250);
}
