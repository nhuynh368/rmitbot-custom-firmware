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

    // Overloaded update function for fusion using wheel velocities (rad/s)
    // Pass in w1, w2, w3, w4 from your Encoder instances
    void update(double w1, double w2, double w3, double w4);

    // Standard update (pure gyro dead-reckoning for yaw)
    void update();

    // Getters for estimated orientation / heading (in degrees)
    float getYaw() const { return yaw; }
    float getPitch() const { return pitch; }
    float getRoll() const { return roll; }

    // Getters for raw/calibrated physical measurements
    float getGx() const { return gx - gx_bias; }
    float getGy() const { return gy - gy_bias; }
    float getGz() const { return gz - gz_bias; }

    // Setter for Mecanum chassis dimensions (meters)
    void setChassisGeometry(float wheelRadius, float lx, float ly) {
        _wheelRadius = wheelRadius;
        _lx = lx;
        _ly = ly;
    }

private:
    uint8_t _addr;
    uint32_t _i2cTimeoutMs;

    // Calibration biases
    float gx_bias = 0.0f, gy_bias = 0.0f, gz_bias = 0.0f;

    // Filtered Output States (Degrees)
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;

    // Raw calibrated values
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;

    // Time tracking for delta-T integration
    unsigned long lastUpdateUs = 0;

    // 1D Kalman Filter Parameters for Yaw Fusion
    float p_yaw = 1.0f;        // Estimate error covariance
    float q_process = 0.005f;  // Process noise covariance (Gyro drift)
    float r_measure = 0.08f;   // Measurement noise covariance (Encoder slippage noise)

    // Robot Geometry Defaults for Mecanum Kinematics (in meters)
    float _wheelRadius = 0.05f; // 50mm radius wheel
    float _lx = 0.15f;          // Center-to-wheel distance (lengthwise)
    float _ly = 0.15f;          // Center-to-wheel distance (widthwise)

    // Low-level safe I2C wrappers
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t *value);
    bool readBytes(uint8_t reg, uint8_t *buffer, size_t length);
    bool selectBank(uint8_t bank);
};

#endif