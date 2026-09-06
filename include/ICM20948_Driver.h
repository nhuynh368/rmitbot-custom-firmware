#ifndef ICM20948_DRIVER_H
#define ICM20948_DRIVER_H
#define IMU_SCL 29 // SCL pin for the IMU
#define IMU_SDA 26 // SDA pin for the IMU

#include <Arduino.h>
#include <Wire.h>

class ICM20948_Driver {
public:
    // Constructor (Default AD0=0 for 0x68 address on WCMCU-20948)
    ICM20948_Driver(uint8_t i2cAddress = 0x68);

    // Initialization routine
    bool begin(TwoWire &wirePort = Wire, int sdaPin = IMU_SDA, int sclPin = IMU_SCL, uint32_t clockSpeed = 400000);

    // Zero-motion static calibration
    void calibrateGyro(int samples = 500);

    // Main update loop - call this frequently in loop()
    bool update();

    // Orientation Getters (Euler Angles in Degrees)
    float getPitch() const { return pitch; }
    float getRoll()  const { return roll; }

    // Raw/Calibrated Rate Getters
    float getGyroX() const { return gx; }
    float getGyroY() const { return gy; }
    float getGyroZ() const { return gz; }

    float getAccelX() const { return ax; }
    float getAccelY() const { return ay; }
    float getAccelZ() const { return az; }

private:
    TwoWire* i2c;
    uint8_t addr;

    // Registers & Limits
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
    static constexpr float FILTER_ALPHA      = 0.98f;

    // Offsets & State Variables
    float gyroBiasX{0.0f}, gyroBiasY{0.0f}, gyroBiasZ{0.0f};
    float pitch{0.0f}, roll{0.0f};
    float ax{0.0f}, ay{0.0f}, az{0.0f};
    float gx{0.0f}, gy{0.0f}, gz{0.0f};
    unsigned long lastMicro{0};

    // Low-Level Bus Helpers
    void writeRegister(uint8_t bank, uint8_t reg, uint8_t val);
    uint8_t readRegister(uint8_t bank, uint8_t reg);
    void readRegisters(uint8_t bank, uint8_t reg, uint8_t* buffer, uint8_t length);
};

#endif // ICM20948_DRIVER_H