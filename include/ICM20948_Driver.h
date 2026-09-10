#ifndef ICM20948_DRIVER_H
#define ICM20948_DRIVER_H

#include <Arduino.h>
#include <Wire.h>

class ICM20948_Driver {
public:
    ICM20948_Driver(uint8_t address = 0x68);

    // Initializes I2C bus, verifies WHO_AM_I, and sets up chip configuration
    bool begin(int sdaPin = 21, int sclPin = 22, uint32_t clockSpeed = 400000);

    // Calibrates gyro static offset safely over a fixed sample count with timeout protection
    bool calibrateGyro(uint16_t samples = 500);

    // Main update loop: Reads Accel/Gyro data, calculates Pitch/Roll, and integrates Yaw
    void update();

    // Getters for calculated orientation / heading (in degrees)
    float getYaw() const { return yaw; }
    float getPitch() const { return pitch; }
    float getRoll() const { return roll; }

    // Getters for calibrated Gyroscope values (deg/s)
    float getGx() const { return gx; }
    float getGy() const { return gy; }
    float getGz() const { return gz; }

    // Getters for Accelerometer values (g)
    float getAx() const { return ax; }
    float getAy() const { return ay; }
    float getAz() const { return az; }

private:
    uint8_t _addr;
    uint32_t _i2cTimeoutMs;

    // Calibration biases for Gyroscope
    float gx_bias = 0.0f, gy_bias = 0.0f, gz_bias = 0.0f;

    // Calculated Orientation States (Degrees)
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;

    // Calibrated sensor readings
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;

    // Time tracking for delta-T integration
    unsigned long lastUpdateUs = 0;

    // Low-level I2C wrappers
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t *value);
    bool readBytes(uint8_t reg, uint8_t *buffer, size_t length);
    bool selectBank(uint8_t bank);
};

#endif