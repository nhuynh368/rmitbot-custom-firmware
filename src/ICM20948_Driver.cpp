#include "ICM20948_Driver.h"
#include <math.h>

// Register definitions
#define REG_BANK_SEL        0x7F
#define REG_WHO_AM_I        0x00
#define REG_PWR_MGMT_1      0x06
#define REG_PWR_MGMT_2      0x07
#define REG_ACCEL_XOUT_H    0x2D
#define REG_GYRO_XOUT_H     0x33

ICM20948_Driver::ICM20948_Driver(uint8_t address) : _addr(address), _i2cTimeoutMs(100) {}

bool ICM20948_Driver::selectBank(uint8_t bank) {
    Wire.beginTransmission(_addr);
    Wire.write(REG_BANK_SEL);
    Wire.write((bank & 0x03) << 4);
    return (Wire.endTransmission() == 0);
}

bool ICM20948_Driver::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

bool ICM20948_Driver::readRegister(uint8_t reg, uint8_t *value) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom(_addr, (uint8_t)1) == 1) {
        *value = Wire.read();
        return true;
    }
    return false;
}

bool ICM20948_Driver::readBytes(uint8_t reg, uint8_t *buffer, size_t length) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    size_t received = Wire.requestFrom(_addr, (uint8_t)length);
    if (received < length) return false;

    for (size_t i = 0; i < length; i++) {
        buffer[i] = Wire.read();
    }
    return true;
}

bool ICM20948_Driver::begin(int sdaPin, int sclPin, uint32_t clockSpeed) {
    Wire.begin(sdaPin, sclPin, clockSpeed);
    Wire.setTimeOut(_i2cTimeoutMs);

    if (!selectBank(0)) return false;

    uint8_t id = 0;
    if (!readRegister(REG_WHO_AM_I, &id) || id != 0xEA) {
        return false;
    }

    if (!writeRegister(REG_PWR_MGMT_1, 0x01)) return false;
    delay(10);
    if (!writeRegister(REG_PWR_MGMT_2, 0x00)) return false;

    lastUpdateUs = micros();
    return true;
}

bool ICM20948_Driver::calibrateGyro(uint16_t samples) {
    float sumGx = 0.0f, sumGy = 0.0f, sumGz = 0.0f;
    uint16_t validSamples = 0;
    unsigned long startTime = millis();

    while (validSamples < samples && (millis() - startTime < 3000)) {
        if (!selectBank(0)) continue;

        uint8_t buf[6];
        if (readBytes(REG_GYRO_XOUT_H, buf, 6)) {
            int16_t rawGx = (int16_t)((buf[0] << 8) | buf[1]);
            int16_t rawGy = (int16_t)((buf[2] << 8) | buf[3]);
            int16_t rawGz = (int16_t)((buf[4] << 8) | buf[5]);

            sumGx += (rawGx / 131.0f);
            sumGy += (rawGy / 131.0f);
            sumGz += (rawGz / 131.0f);
            validSamples++;
        }
        delay(2);
    }

    if (validSamples == 0) return false;

    gx_bias = sumGx / validSamples;
    gy_bias = sumGy / validSamples;
    gz_bias = sumGz / validSamples;
    return true;
}

void ICM20948_Driver::update() {
    unsigned long nowUs = micros();
    float dt = (nowUs - lastUpdateUs) * 1e-6f;
    lastUpdateUs = nowUs;

    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

    if (!selectBank(0)) return;

    // Read 12 bytes: Accel (6 bytes: X, Y, Z) + Gyro (6 bytes: X, Y, Z)
    uint8_t rawData[12];
    if (!readBytes(REG_ACCEL_XOUT_H, rawData, 12)) return;

    // Parse Accelerometer (Default ±2g scale factor: 16384 LSB/g)
    int16_t rawAx = (int16_t)((rawData[0] << 8) | rawData[1]);
    int16_t rawAy = (int16_t)((rawData[2] << 8) | rawData[3]);
    int16_t rawAz = (int16_t)((rawData[4] << 8) | rawData[5]);

    ax = rawAx / 16384.0f;
    ay = rawAy / 16384.0f;
    az = rawAz / 16384.0f;

    // Parse Gyroscope (Default ±250 dps scale factor: 131 LSB/dps)
    int16_t rawGx = (int16_t)((rawData[6] << 8) | rawData[7]);
    int16_t rawGy = (int16_t)((rawData[8] << 8) | rawData[9]);
    int16_t rawGz = (int16_t)((rawData[10] << 8) | rawData[11]);

    gx = (rawGx / 131.0f) - gx_bias;
    gy = (rawGy / 131.0f) - gy_bias;
    gz = (rawGz / 131.0f) - gz_bias;

    // Pitch and Roll derived purely from Accelerometer
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    roll  = atan2f(ay, az) * RAD_TO_DEG;

    // Yaw calculated strictly via Z-axis Gyroscope integration
    yaw += gz * dt;

    // Keep heading bounded within [-180, 180] degrees
    if (yaw > 180.0f) yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;
}