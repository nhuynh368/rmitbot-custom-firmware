#ifndef ICM20948_DRIVER_H
#define ICM20948_DRIVER_H

#include <Arduino.h>
#include <Wire.h>

class ICM20948_Driver {
public:
    // Constructor (Default AD0=0 for 0x68 address on WCMCU-20948)
    explicit ICM20948_Driver(uint8_t i2cAddress = 0x68);

    // Initialization routine
    bool begin(TwoWire &wirePort = Wire, int sdaPin = 21, int sclPin = 22, uint32_t clockSpeed = 400000);

    // Zero-motion static gyro calibration
    void calibrateGyro(int samples = 500);

    // Main update loop - run continuously in main loop()
    bool update();

    // Tuning method for Kalman Filter parameters
    void setKalmanTuning(float Q_angle, float Q_bias, float R_measure);

    // Orientation Getters (Euler Angles in Degrees)
    float getPitch() const { return pitch; }
    float getRoll()  const { return roll; }

    // Sensor Rate & Acceleration Getters
    float getGyroX() const { return gx; }
    float getGyroY() const { return gy; }
    float getGyroZ() const { return gz; }

    float getAccelX() const { return ax; }
    float getAccelY() const { return ay; }
    float getAccelZ() const { return az; }

private:
    TwoWire* i2c;
    uint8_t addr;

    // ICM-20948 Registers
    static constexpr uint8_t REG_BANK_SEL      = 0x7F;
    static constexpr uint8_t REG_WHO_AM_I      = 0x00;
    static constexpr uint8_t REG_PWR_MGMT_1    = 0x06;
    static constexpr uint8_t REG_PWR_MGMT_2    = 0x07;
    static constexpr uint8_t REG_ACCEL_CONFIG  = 0x14;
    static constexpr uint8_t REG_GYRO_CONFIG_1 = 0x01;
    static constexpr uint8_t REG_ACCEL_XOUT_H  = 0x2D;
    static constexpr uint8_t REG_GYRO_XOUT_H   = 0x33;

    // Sensitivities (±2000 dps, ±2g defaults)
    static constexpr float GYRO_SENSITIVITY  = 16.4f;
    static constexpr float ACCEL_SENSITIVITY = 16384.0f;

    // Offsets & Sensor Values
    float gyroBiasX{0.0f}, gyroBiasY{0.0f}, gyroBiasZ{0.0f};
    float ax{0.0f}, ay{0.0f}, az{0.0f};
    float gx{0.0f}, gy{0.0f}, gz{0.0f};
    float pitch{0.0f}, roll{0.0f};
    unsigned long lastMicro{0};

    // --- Kalman Filter Struct for single-axis estimation ---
    struct KalmanState {
        float Q_angle{0.001f};   // Process noise for accelerometer/angle state
        float Q_bias{0.003f};    // Process noise for gyro bias estimation
        float R_measure{0.03f};  // Measurement noise covariance (motor vibration)

        float angle{0.0f};       // Estimated angle
        float bias{0.0f};        // Estimated gyro bias

        float P[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}}; // Covariance matrix
    };

    KalmanState kalmanPitch;
    KalmanState kalmanRoll;

    // Step calculation function for Kalman Filter
    float computeKalman(KalmanState &k, float newAngle, float newRate, float dt);

    // Low-Level Bus Helpers
    void writeRegister(uint8_t bank, uint8_t reg, uint8_t val);
    uint8_t readRegister(uint8_t bank, uint8_t reg);
    void readRegisters(uint8_t bank, uint8_t reg, uint8_t* buffer, uint8_t length);
};

#endif // ICM20948_DRIVER_H