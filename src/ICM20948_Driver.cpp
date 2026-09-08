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
    // 1. Initialize Wire bus
    Wire.begin(sdaPin, sclPin, clockSpeed);
    Wire.setTimeOut(_i2cTimeoutMs); // Critical timeout guard

    // 2. Switch to User Bank 0
    if (!selectBank(0)) return false;

    // 3. Verify WHO_AM_I signature (0xEA)
    uint8_t id = 0;
    if (!readRegister(REG_WHO_AM_I, &id) || id != 0xEA) {
        return false;
    }

    // 4. Wake up device (Clear SLEEP bit)
    if (!writeRegister(REG_PWR_MGMT_1, 0x01)) return false; // Auto select clock
    delay(10);
    if (!writeRegister(REG_PWR_MGMT_2, 0x00)) return false; // Enable Accel & Gyro

    lastUpdateUs = micros();
    return true;
}

bool ICM20948_Driver::calibrateGyro(uint16_t samples) {
    float sumGx = 0.0f, sumGy = 0.0f, sumGz = 0.0f;
    uint16_t validSamples = 0;
    unsigned long startTime = millis();

    // Prevent infinite loop if I2C glitches during calibration (max 3 sec runtime)
    while (validSamples < samples && (millis() - startTime < 3000)) {
        if (!selectBank(0)) continue;

        uint8_t buf[6];
        if (readBytes(REG_GYRO_XOUT_H, buf, 6)) {
            int16_t rawGx = (int16_t)((buf[0] << 8) | buf[1]);
            int16_t rawGy = (int16_t)((buf[2] << 8) | buf[3]);
            int16_t rawGz = (int16_t)((buf[4] << 8) | buf[5]);

            // Full scale range sensitivity factor for +/- 250 dps (131.0 LSB/dps)
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

    // Cap delta-T to prevent matrix explosion after pause or startup
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

    if (!selectBank(0)) return;

    // Read 12 contiguous bytes: Accel (6) + Gyro (6)
    uint8_t rawData[12];
    if (!readBytes(REG_ACCEL_XOUT_H, rawData, 12)) {
        return; // Safely abort without changing states if I2C drops frames
    }

    // Parse Accelerometer (LSB to g force assuming default +/- 2g range = 16384 LSB/g)
    int16_t rawAx = (int16_t)((rawData[0] << 8) | rawData[1]);
    int16_t rawAy = (int16_t)((rawData[2] << 8) | rawData[3]);
    int16_t rawAz = (int16_t)((rawData[4] << 8) | rawData[5]);

    ax = rawAx / 16384.0f;
    ay = rawAy / 16384.0f;
    az = rawAz / 16384.0f;

    // Parse Gyroscope (LSB to dps assuming default +/- 250 dps range = 131 LSB/dps)
    int16_t rawGx = (int16_t)((rawData[6] << 8) | rawData[7]);
    int16_t rawGy = (int16_t)((rawData[8] << 8) | rawData[9]);
    int16_t rawGz = (int16_t)((rawData[10] << 8) | rawData[11]);

    gx = (rawGx / 131.0f) - gx_bias;
    gy = (rawGy / 131.0f) - gy_bias;
    gz = (rawGz / 131.0f) - gz_bias;

    // Prevent division by zero FPU crashes
    float accelNorm = sqrtf(ax * ax + ay * ay + az * az);
    if (accelNorm < 0.1f) return;

    // -------------------------------------------------------------
    // EXTENDED KALMAN FILTER (EKF) IMPLEMENTATION (2-State)
    // -------------------------------------------------------------

    // 1. PREDICT STEP
    // Kinematic propagation using Gyro rates
    pitch += gy * dt;
    roll  += gx * dt;

    // Propagate state covariance: P_k|k-1 = F * P_k-1|k-1 * F^T + Q
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_gyro * dt;

    // 2. MEASURE STEP (Calculate Pitch/Roll directly from Accelerometer)
    float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    float roll_acc  = atan2f(ay, az) * RAD_TO_DEG;

    // 3. UPDATE STEP (Pitch State)
    float y_pitch = pitch_acc - pitch;          // Innovation residual
    float S_pitch = P[0][0] + R_angle;          // Innovation covariance
    if (S_pitch > 1e-6f) {
        float K_pitch[2] = { P[0][0] / S_pitch, P[1][0] / S_pitch }; // Kalman gain
        pitch += K_pitch[0] * y_pitch;
        P[0][0] -= K_pitch[0] * P[0][0];
        P[0][1] -= K_pitch[0] * P[0][1];
    }

    // UPDATE STEP (Roll State)
    float y_roll = roll_acc - roll;             // Innovation residual
    float S_roll = P[1][1] + R_angle;           // Innovation covariance
    if (S_roll > 1e-6f) {
        float K_roll[2] = { P[0][1] / S_roll, P[1][1] / S_roll };   // Kalman gain
        roll += K_roll[1] * y_roll;
        P[1][0] -= K_roll[1] * P[1][0];
        P[1][1] -= K_roll[1] * P[1][1];
    }
}