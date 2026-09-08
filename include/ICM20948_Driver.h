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

    // Reads raw sensor data and executes the 2-state EKF update step
    void update();

    // Getters for estimated orientation (in degrees)
    float getPitch() const { return pitch; }
    float getRoll() const { return roll; }

    // Getters for raw/calibrated physical measurements
    float getGx() const { return gx - gx_bias; }
    float getGy() const { return gy - gy_bias; }
    float getGz() const { return gz - gz_bias; }

private:
    uint8_t _addr;
    uint32_t _i2cTimeoutMs;

    // Calibration biases
    float gx_bias = 0.0f, gy_bias = 0.0f, gz_bias = 0.0f;

    // Filtered Output States (Degrees)
    float pitch = 0.0f;
    float roll = 0.0f;

    // Raw calibrated values
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;

    // Time tracking for delta-T integration
    unsigned long lastUpdateUs = 0;

    // 2-State EKF Covariance Matrices (State vector: x = [pitch, roll]^T)
    float P[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}}; // Error covariance
    float Q_angle = 0.001f;                        // Process noise covariance (accel trust)
    float Q_gyro  = 0.003f;                        // Process noise covariance (gyro drift trust)
    float R_angle = 0.03f;                         // Measurement noise covariance

    // Low-level safe I2C wrappers
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t *value);
    bool readBytes(uint8_t reg, uint8_t *buffer, size_t length);
    bool selectBank(uint8_t bank);
};

#endif