#include <Arduino.h> // Arduino library for basic functions

#include "ICM20948_Driver.h" // Library for the IMU
#include "MySetup.h"      // Pin and variable definitions
#include "MyEncoder.h"    // Library for the encoder
#include "MyMotor.h"      // Library for the motor
#include "MyController.h" // Library for the controller
#include "MySerial.h"     // Library for the controller

ICM20948_Driver imu(0x68); // Create an instance of the ICM20948_Driver class with the default I2C address

Encoder encoder1(ENC1_A, ENC1_B);                // Create an instance of the Encoder class
Encoder encoder2(ENC2_A, ENC2_B);                // Create an instance of the Encoder class
Encoder encoder3(ENC3_A, ENC3_B);                // Create an instance of the Encoder class
Encoder encoder4(ENC4_A, ENC4_B);                // Create an instance of the Encoder class
Motor motor1(MOT1_A, MOT1_B, PWM1_A, PWM1_B);    // Create an instance of the Motor class
Motor motor2(MOT2_A, MOT2_B, PWM2_A, PWM2_B);    // Create an instance of the Motor class
Motor motor3(MOT3_A, MOT3_B, PWM3_A, PWM3_B);    // Create an instance of the Motor class
Motor motor4(MOT4_A, MOT4_B, PWM4_A, PWM4_B);    // Create an instance of the Motor class
Controller controller1(&w1, &MOT1_cmd, &w1_ref); // Create an instance of the Controller class
Controller controller2(&w2, &MOT2_cmd, &w2_ref); // Create an instance of the Controller class
Controller controller3(&w3, &MOT3_cmd, &w3_ref); // Create an instance of the Controller class
Controller controller4(&w4, &MOT4_cmd, &w4_ref); // Create an instance of the Controller class

//==============================================
// Arduino Setup
//==============================================
void setup()
{
  encoder1.begin();    // Initialize the encoder
  encoder2.begin();    // Initialize the encoder
  encoder3.begin();    // Initialize the encoder
  encoder4.begin();    // Initialize the encoder
  motor1.begin();      // Initialize the motor
  motor2.begin();      // Initialize the motor
  motor3.begin();      // Initialize the motor
  motor4.begin();      // Initialize the motor
  controller1.begin(); // Initialize the controller
  controller2.begin(); // Initialize the controller
  controller3.begin(); // Initialize the controller
  controller4.begin(); // Initialize the controller
  SerialBegin();       // Initialize the serial communication
  // imu setup
  if (!imu.begin(IMU_SDA, IMU_SCL)) {
    Serial.println("IMU init failed");
    while (1) delay(100); // watch loop waiting for IMU to be connected
  }

  Serial.println("IMU calibrating, hold still...");
  imu.calibrateGyro(500);
  Serial.println("IMU calibrated");
}

void loop()
{
  EncoderTick1 = encoder1.getCount(); // Get the encoder count
  EncoderTick2 = encoder2.getCount(); // Get the encoder count
  EncoderTick3 = encoder3.getCount(); // Get the encoder count
  EncoderTick4 = encoder4.getCount(); // Get the encoder count
  
  w1 = encoder1.getVelocity();        // Get the velocity from the encoder
  w2 = encoder2.getVelocity();        // Get the velocity from the encoder
  w3 = encoder3.getVelocity();        // Get the velocity from the encoder
  w4 = encoder4.getVelocity();        // Get the velocity from the encoder
  
  imu.update(w1,w2,w3,w4);                       // Update the IMU readings
  
  controller1.compute();              // Compute the PID control output
  controller2.compute();              // Compute the PID control output
  controller3.compute();              // Compute the PID control output
  controller4.compute();              // Compute the PID control output
  
  motor1.send_pwm(MOT1_cmd);          // Send the PWM command to the motor
  motor2.send_pwm(MOT2_cmd);          // Send the PWM command to the motor
  motor3.send_pwm(MOT3_cmd);          // Send the PWM command to the motor
  motor4.send_pwm(MOT4_cmd);          // Send the PWM command to the motor
  
  SerialDataPrint();                  // Print the data to the Serial Monitor
  SerialDataRead();                  // Write the data to the Serial Monitor
}
