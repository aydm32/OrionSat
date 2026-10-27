/*
 * E-Cube (ESP32) sensor streamer, PlatformIO / Arduino framework.
 *
 * Scans the I2C bus at boot and enables whatever it recognises:
 *   accel+gyro : MPU6050 / MPU6500 / MPU9250 (0x68 or 0x69)      -> ax ay az gx gy gz
 *   accel only : ADXL345 (0x53)                                   -> ax ay az
 *   magnetometer: HMC5883L (0x1E) or QMC5883L (0x0D)              -> mx my mz  (uT)
 *   barometer  : BMP180 (0x77)                                    -> pres (hPa) temp (C) alt (m, relative)
 * Missing channels are sent as "nan".
 *
 * Output, 100 Hz, one line per sample (USB serial, 460800 baud):
 *   t_ms,ax,ay,az,gx,gy,gz,mx,my,mz,pres,temp,alt
 * Lines starting with '#' are informational (what was found, status).
 *
 * Keep the board still for ~2 s after reset: gyro bias calibration.
 *
 * I2C pins: ESP32 defaults GPIO21 (SDA) / GPIO22 (SCL). If your E-Cube mission board
 * uses other pins, change I2C_SDA / I2C_SCL below. The boot log lists every address found.
 */
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define I2C_SDA 21
#define I2C_SCL 22
static const uint32_t SERIAL_BAUD      = 460800;
static const uint16_t SAMPLE_PERIOD_MS = 10;     // 100 Hz
static const uint16_t BARO_PERIOD_MS   = 200;    // 5 Hz

// ------------------------------------------------------------ I2C helpers
static bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static uint8_t readReg(uint8_t addr, uint8_t reg) {
  uint8_t v = 0xFF;
  readRegs(addr, reg, &v, 1);
  return v;
}

// ------------------------------------------------------------ sensor state
enum ImuType { IMU_NONE, IMU_MPU, IMU_ADXL };
enum MagType { MAG_NONE, MAG_HMC, MAG_QMC };

static ImuType imuType = IMU_NONE;
static MagType magType = MAG_NONE;
static uint8_t imuAddr = 0;
static bool    hasBaro = false;
static const uint8_t HMC_ADDR = 0x1E, QMC_ADDR = 0x0D, BMP_ADDR = 0x77, ADXL_ADDR = 0x53;

static float gxOff = 0, gyOff = 0, gzOff = 0;      // gyro bias, deg/s

// ------------------------------------------------------------ MPU6050 family
static const float MPU_ACC_SENS = 8192.0f;         // LSB/g  (+-4 g)
static const float MPU_GYR_SENS = 65.5f;           // LSB/dps (+-500 dps)

static void mpuInit() {
  writeReg(imuAddr, 0x6B, 0x01);   // wake, PLL clock
  writeReg(imuAddr, 0x1A, 0x03);   // DLPF ~44 Hz
  writeReg(imuAddr, 0x1B, 0x08);   // gyro  +-500 dps
  writeReg(imuAddr, 0x1C, 0x08);   // accel +-4 g
  delay(100);
}

static bool mpuRead(float acc[3], float gyr[3]) {
  uint8_t b[14];
  if (!readRegs(imuAddr, 0x3B, b, 14)) return false;
  auto s16 = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };
  acc[0] = s16(0) / MPU_ACC_SENS;
  acc[1] = s16(2) / MPU_ACC_SENS;
  acc[2] = s16(4) / MPU_ACC_SENS;
  gyr[0] = s16(8)  / MPU_GYR_SENS - gxOff;
  gyr[1] = s16(10) / MPU_GYR_SENS - gyOff;
  gyr[2] = s16(12) / MPU_GYR_SENS - gzOff;
  return true;
}

static void mpuCalibrate() {
  const uint16_t N = 400;
  float sx = 0, sy = 0, sz = 0, a[3], g[3];
  uint16_t n = 0;
  while (n < N) {
    if (mpuRead(a, g)) { sx += g[0]; sy += g[1]; sz += g[2]; n++; }
    delay(4);
  }
  // mpuRead already subtracts the (zero) offsets, so the mean is the bias
  gxOff = sx / n; gyOff = sy / n; gzOff = sz / n;
}

// ------------------------------------------------------------ ADXL345
static void adxlInit() {
  writeReg(ADXL_ADDR, 0x2C, 0x0A);   // 100 Hz
  writeReg(ADXL_ADDR, 0x31, 0x08);   // full resolution, +-2 g
  writeReg(ADXL_ADDR, 0x2D, 0x08);   // measure
}

static bool adxlRead(float acc[3]) {
  uint8_t b[6];
  if (!readRegs(ADXL_ADDR, 0x32, b, 6)) return false;
  for (int i = 0; i < 3; i++) acc[i] = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8)) / 256.0f;
  return true;
}

// ------------------------------------------------------------ magnetometers (output in uT)
static void hmcInit() {
  writeReg(HMC_ADDR, 0x00, 0x78);    // 8x average, 75 Hz
  writeReg(HMC_ADDR, 0x01, 0x20);    // +-1.3 Ga -> 1090 LSB/Ga
  writeReg(HMC_ADDR, 0x02, 0x00);    // continuous
}

static bool hmcRead(float m[3]) {
  uint8_t b[6];
  if (!readRegs(HMC_ADDR, 0x03, b, 6)) return false;
  int16_t x = (int16_t)((b[0] << 8) | b[1]);
  int16_t z = (int16_t)((b[2] << 8) | b[3]);   // order is X, Z, Y
  int16_t y = (int16_t)((b[4] << 8) | b[5]);
  if (x == -4096 || y == -4096 || z == -4096) return false;   // overflow
  const float k = 100.0f / 1090.0f;
  m[0] = x * k; m[1] = y * k; m[2] = z * k;
  return true;
}

static void qmcInit() {
  writeReg(QMC_ADDR, 0x0A, 0x80);    // soft reset
  delay(10);
  writeReg(QMC_ADDR, 0x0B, 0x01);    // set/reset period
  writeReg(QMC_ADDR, 0x09, 0x1D);    // OSR 512, +-8 G, 100 Hz, continuous
}

static bool qmcRead(float m[3]) {
  uint8_t b[6];
  if (!readRegs(QMC_ADDR, 0x00, b, 6)) return false;
  const float k = 100.0f / 3000.0f;  // +-8 G -> 3000 LSB/G
  m[0] = (int16_t)(b[0] | (b[1] << 8)) * k;
  m[1] = (int16_t)(b[2] | (b[3] << 8)) * k;
  m[2] = (int16_t)(b[4] | (b[5] << 8)) * k;
  return true;
}

// ------------------------------------------------------------ BMP180 (non-blocking)
static int16_t  AC1, AC2, AC3, B1, B2, MB, MC, MD;
static uint16_t AC4, AC5, AC6;
static const uint8_t BMP_OSS = 1;
static float pressHpa = NAN, tempC = NAN, altM = NAN, p0Pa = 0;
static uint8_t baroState = 0;
static uint32_t baroT = 0;
static int32_t rawUT = 0;
static int baroCount = 0;

static uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

static bool bmpInit() {
  uint8_t c[22];
  if (!readRegs(BMP_ADDR, 0xAA, c, 22)) return false;
  AC1 = be16(c); AC2 = be16(c + 2); AC3 = be16(c + 4);
  AC4 = be16(c + 6); AC5 = be16(c + 8); AC6 = be16(c + 10);
  B1 = be16(c + 12); B2 = be16(c + 14); MB = be16(c + 16);
  MC = be16(c + 18); MD = be16(c + 20);
  return true;
}

static void bmpCompute(int32_t UT, int32_t UP) {
  int32_t X1 = ((UT - (int32_t)AC6) * (int32_t)AC5) >> 15;
  int32_t X2 = ((int32_t)MC * 2048) / (X1 + MD);
  int32_t B5 = X1 + X2;
  tempC = ((B5 + 8) >> 4) / 10.0f;

  int32_t B6 = B5 - 4000;
  X1 = ((int32_t)B2 * ((B6 * B6) >> 12)) >> 11;
  X2 = ((int32_t)AC2 * B6) >> 11;
  int32_t X3 = X1 + X2;
  int32_t B3 = ((((int32_t)AC1 * 4 + X3) * (1 << BMP_OSS)) + 2) / 4;
  X1 = ((int32_t)AC3 * B6) >> 13;
  X2 = ((int32_t)B1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  uint32_t B4 = ((uint32_t)AC4 * (uint32_t)(X3 + 32768)) >> 15;
  uint32_t B7 = ((uint32_t)UP - B3) * (50000 >> BMP_OSS);
  int32_t p = (B7 < 0x80000000UL) ? (int32_t)((B7 * 2) / B4) : (int32_t)((B7 / B4) * 2);
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;
  p = p + ((X1 + X2 + 3791) >> 4);

  pressHpa = p / 100.0f;
  if (baroCount < 10) {                       // reference = mean of first readings
    p0Pa = (p0Pa * baroCount + p) / (baroCount + 1);
    baroCount++;
  }
  altM = 44330.0f * (1.0f - powf((float)p / p0Pa, 0.190295f));
}

static void baroStep() {
  uint32_t now = millis();
  switch (baroState) {
    case 0:
      if (now - baroT >= BARO_PERIOD_MS) {
        writeReg(BMP_ADDR, 0xF4, 0x2E);
        baroT = now; baroState = 1;
      }
      break;
    case 1:
      if (now - baroT >= 6) {
        uint8_t b[2];
        if (readRegs(BMP_ADDR, 0xF6, b, 2)) rawUT = be16(b);
        writeReg(BMP_ADDR, 0xF4, 0x34 + (BMP_OSS << 6));
        baroT = now; baroState = 2;
      }
      break;
    case 2:
      if (now - baroT >= 9) {
        uint8_t b[3];
        if (readRegs(BMP_ADDR, 0xF6, b, 3)) {
          int32_t UP = (((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | b[2]) >> (8 - BMP_OSS);
          bmpCompute(rawUT, UP);
        }
        baroT = now; baroState = 0;
      }
      break;
  }
}

// ------------------------------------------------------------ setup / loop
static void detectSensors() {
  char found[160];
  size_t n = 0;
  found[0] = 0;

  Serial.print("# i2c addresses:");
  for (uint8_t a = 1; a < 127; a++)
    if (i2cPresent(a)) { Serial.print(" 0x"); Serial.print(a, HEX); }
  Serial.println();

  for (uint8_t a = 0x68; a <= 0x69 && imuType == IMU_NONE; a++) {
    uint8_t id = readReg(a, 0x75);
    if (id == 0x68 || id == 0x70 || id == 0x71 || id == 0x73) {
      imuType = IMU_MPU; imuAddr = a;
      n += snprintf(found + n, sizeof(found) - n, "MPU(id=0x%02X)@0x%02X;", id, a);
    }
  }
  if (imuType == IMU_NONE && i2cPresent(ADXL_ADDR) && readReg(ADXL_ADDR, 0x00) == 0xE5) {
    imuType = IMU_ADXL;
    n += snprintf(found + n, sizeof(found) - n, "ADXL345@0x53;");
  }
  if (i2cPresent(HMC_ADDR) && readReg(HMC_ADDR, 0x0A) == 'H' && readReg(HMC_ADDR, 0x0B) == '4') {
    magType = MAG_HMC;
    n += snprintf(found + n, sizeof(found) - n, "HMC5883L@0x1E;");
  } else if (i2cPresent(QMC_ADDR) && readReg(QMC_ADDR, 0x0D) == 0xFF) {
    magType = MAG_QMC;
    n += snprintf(found + n, sizeof(found) - n, "QMC5883L@0x0D;");
  }
  if (i2cPresent(BMP_ADDR)) {
    uint8_t id = readReg(BMP_ADDR, 0xD0);
    if (id == 0x55 && bmpInit()) {
      hasBaro = true;
      n += snprintf(found + n, sizeof(found) - n, "BMP180@0x77;");
    } else {
      Serial.print("# unsupported pressure sensor at 0x77 chip-id 0x");
      Serial.println(id, HEX);
    }
  }
  Serial.print("# found ;");
  Serial.println(found);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);
  Serial.println("# E-CUBE streamer 1.0");
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Serial.print("# I2C SDA="); Serial.print(I2C_SDA);
  Serial.print(" SCL="); Serial.println(I2C_SCL);

  detectSensors();

  if (imuType == IMU_MPU) {
    mpuInit();
    Serial.println("# calibrating gyro; keep the cube still");
    mpuCalibrate();
  } else if (imuType == IMU_ADXL) {
    adxlInit();
    Serial.println("# accelerometer only (no gyro): viewer will use tilt + compass");
  } else {
    Serial.println("# WARNING no accelerometer found; check I2C pins / wiring");
  }
  if (magType == MAG_HMC) hmcInit();
  if (magType == MAG_QMC) qmcInit();

  Serial.println("# fields t_ms ax ay az gx gy gz mx my mz pres temp alt");
  Serial.println("# READY");
}

void loop() {
  static uint32_t next = millis();
  if (hasBaro) baroStep();
  if ((int32_t)(millis() - next) < 0) return;
  next += SAMPLE_PERIOD_MS;

  float acc[3] = {NAN, NAN, NAN}, gyr[3] = {NAN, NAN, NAN}, mag[3] = {NAN, NAN, NAN};
  bool ok = false;
  if (imuType == IMU_MPU)       ok = mpuRead(acc, gyr);
  else if (imuType == IMU_ADXL) ok = adxlRead(acc);
  if (!ok) { if (imuType != IMU_NONE) return; }

  if (magType == MAG_HMC) hmcRead(mag);   // keeps NaN on failure
  if (magType == MAG_QMC) qmcRead(mag);

  char line[200];
  snprintf(line, sizeof(line), "%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
           (unsigned long)millis(), acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2],
           mag[0], mag[1], mag[2], pressHpa, tempC, altM);
  Serial.println(line);
}
