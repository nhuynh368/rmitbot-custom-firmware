#include "ICM20948_Driver.h"

ICM20948_Driver::ICM20948_Driver(uint8_t i2cAddress) : addr(i2cAddress) {}

bool ICM20948_Driver::begin(TwoWire &wirePort, int sdaPin, int sclPin, uint32_t clockSpeed) {
    i2c = &wirePort;
    i2c->begin(sdaPin, sclPin);
    i2c->setClock(clockSpeed);

    // Verify Chip ID (Bank 0, REG_WHO_AM_I should be 0xEA)
    uint8_t chipID = readRegister(0, REG_WHO_AM_I);
    if (chipID != 0xEA) {
        return false;
    }

    // Wake up chip & set auto-select clock source (Bank 0)
    writeRegister(0, REG_PWR_MGMT_1, 0x01);
    delay(10);

    // Enable Gyroscope and Accelerometer (Bank 0)
    writeRegister(0, REG_PWR_MGMT_2, 0x00);

    // Bank 2 Config: Enable Gyro DLPF (~24Hz Cutoff) & Set ±2000 dps full scale
    writeRegister(2, REG_GYRO_CONFIG_1, (0x03 << 3) | (0x03 << 1) | 0x01);

    // Bank 2 Config: Enable Accel DLPF (~24Hz Cutoff) & Set ±2g full scale
    writeRegister(2, REG_ACCEL_CONFIG, (0x03 << 3) | (0x00 << 1) | 0x01);

    // Return to Bank 0 for continuous reading
    writeRegister(0, REG_BANK_SEL, 0x00);

    lastMicro = micros();
    return true;
}

void ICM20948_Driver::calibrateGyro(int samples) {
    long sumX = 0, sumY = 0, sumZ = 0;
    uint8_t rawData[6];

    for (int i = 0; i < samples; i++) {
        readRegisters(0, REG_GYRO_XOUT_H, rawData, 6);
        sumX += (int16_t)((rawData[0] << 8) | rawData[1]);
        sumY += (int16_t)((rawData[2] << 8) | rawData[3]);
        sumZ += (int16_t)((rawData[4] << 8) | rawData[5]);
        delay(3);
    }

    gyroBiasX = ((float)sumX / samples) / GYRO_SENSITIVITY;
    gyroBiasY = ((float)sumY / samples) / GYRO_SENSITIVITY;
    gyroBiasZ = ((float)sumZ / samples) / GYRO_SENSITIVITY;
}

bool ICM20948_Driver::update() {
    uint8_t rawBuffer[12]; // Accel (6 bytes) + Gyro (6 bytes)

    // Single burst read over Bank 0
    readRegisters(0, REG_ACCEL_XOUT_H, rawBuffer, 12);

    // Microsecond timing calculation
    unsigned long currentMicro = micros();
    float dt = (currentMicro - lastMicro) / 1000000.0f;
    lastMicro = currentMicro;

    if (dt <= 0.0f) return false;

    // Convert raw 16-bit signed integers
    int16_t rawAX = (rawBuffer[0] << 8) | rawBuffer[1];
    int16_t rawAY = (rawBuffer[2] << 8) | rawBuffer[3];
    int16_t rawAZ = (rawBuffer[4] << 8) | rawBuffer[5];

    int16_t rawGX = (rawBuffer[6] << 8) | rawBuffer[7];
    int16_t rawGY = (rawBuffer[8] << 8) | rawBuffer[9];
    int16_t rawGZ = (rawBuffer[10] << 8) | rawBuffer[11];

    // Scale to physical units & subtract static offset
    ax = rawAX / ACCEL_SENSITIVITY;
    ay = rawAY / ACCEL_SENSITIVITY;
    az = rawAZ / ACCEL_SENSITIVITY;

    gx = (rawGX / GYRO_SENSITIVITY) - gyroBiasX;
    gy = (rawGY / GYRO_SENSITIVITY) - gyroBiasY;
    gz = (rawGZ / GYRO_SENSITIVITY) - gyroBiasZ;

    // Accelerometer static orientation reference
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * (180.0f / PI);
    float accelRoll  = atan2(ay, az) * (180.0f / PI);

    // 6-Axis Complementary Filter
    pitch = FILTER_ALPHA * (pitch + gx * dt) + (1.0f - FILTER_ALPHA) * accelPitch;
    roll  = FILTER_ALPHA * (roll  + gy * dt) + (1.0f - FILTER_ALPHA) * accelRoll;

    return true;
}

// Low-Level Bus Helpers
void ICM20948_Driver::writeRegister(uint8_t bank, uint8_t reg, uint8_t val) {
    i2c->beginTransmission(addr);
    i2c->write(REG_BANK_SEL);
    i2c->write(bank << 4);
    i2c->endTransmission();

    i2c->beginTransmission(addr);
    i2c->write(reg);
    i2c->write(val);
    i2c->endTransmission();
}

uint8_t ICM20948_Driver::readRegister(uint8_t bank, uint8_t reg) {
    i2c->beginTransmission(addr);
    i2c->write(REG_BANK_SEL);
    i2c->write(bank << 4);
    i2c->endTransmission();

    i2c->beginTransmission(addr);
    i2c->write(reg);
    i2c->requestFrom(addr, (uint8_t)1);
    return i2c->available() ? i2c->read() : 0x00;
}

void ICM20948_Driver::readRegisters(uint8_t bank, uint8_t reg, uint8_t* buffer, uint8_t length) {
    i2c->beginTransmission(addr);
    i2c->write(REG_BANK_SEL);
    i2c->write(bank << 4);
    i2c->endTransmission();

    i2c->beginTransmission(addr);
    i2c->write(reg);
    i2c->endTransmission(false); // Repeated start

    i2c->requestFrom(addr, length);
    for (uint8_t i = 0; i < length && i2c->available(); i++) {
        buffer[i] = i2c->read();
    }
}