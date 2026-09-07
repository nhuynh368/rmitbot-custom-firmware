#include "MySerial.h"
#include "ICM20948_Driver.h"
#include <vector>
#include <string>

extern ICM20948_Driver imu; // Refernce to the IMU driver instance defined in main.cpp
extern double w1, w1_ref, MOT1_cmd; // Reference and command for the motor 1 - defined in MySetup.h
extern double w2, w2_ref, MOT2_cmd; // Reference and command for the motor 2 - defined in MySetup.h
extern double w3, w3_ref, MOT3_cmd; // Reference and command for the motor 1 - defined in MySetup.h
extern double w4, w4_ref, MOT4_cmd; // Reference and command for the motor 1 - defined in MySetup.h
extern volatile long EncoderTick1;  // Encoder tick count for encoder 1 - defined in MySetup.h
extern volatile long EncoderTick2;  // Encoder tick count for encoder 2 - defined in MySetup.h
extern volatile long EncoderTick3;    // Encoder tick count for encoder 3
extern volatile long EncoderTick4;    // Encoder tick count for encoder 4
extern unsigned long Serial_time;   // Time for serial communication - defined in MySetup.h
String incomingMessage = "";
bool receiving = false;

void SerialBegin() // Function to initialize the serial communication
{
    Serial.begin(115200);
    while (!Serial);
}

void SerialDataPrint() // Function to print the data to the Serial Monitor
{
    if (micros() - Serial_time >= 50 * 1e3)
    {
        Serial_time = micros();
        Serial.print('<');
        Serial.print(w1);
        Serial.print("\t");
        Serial.print(w2);
        Serial.print("\t");
        Serial.print(w3);
        Serial.print("\t");
        Serial.print(w4);

        // imu data
        Serial.print("\t");
        Serial.print(imu.getPitch(), 2);  // Pitch angle in degrees
        Serial.print("\t");
        Serial.print(imu.getRoll(), 2);   // Roll angle in degrees
        Serial.print("\t");
        Serial.print(imu.getGyroZ(), 2);  // Yaw angular velocity (deg/s)
        
        Serial.println('>');
    }
}

void SerialDataRead()
{
    while (Serial.available() > 0)
    {
        char c = Serial.read();

        if (c == '<')
        {
            receiving = true;
            incomingMessage = ""; // Reset buffer
        }
        else if (c == '>')
        {
            receiving = false;
            parseCommand(incomingMessage);
        }
        else if (receiving)
        {
            incomingMessage += c;
        }
    }
}

void parseCommand(const String &msg)
{
   
    std::vector<int> tabIndices;
    for (int i = 0; i < msg.length(); ++i) {
        if (msg[i] == '\t') {tabIndices.push_back(i);}
    }

    String w1_str = msg.substring(0, tabIndices[0] - 1);
    String w2_str = msg.substring(tabIndices[0], tabIndices[1] - 1);
    String w3_str = msg.substring(tabIndices[1], tabIndices[2] - 1);
    String w4_str = msg.substring(tabIndices[2], tabIndices[3] - 1);
    w1_ref = w1_str.toFloat();
    w2_ref = w2_str.toFloat();
    w3_ref = w3_str.toFloat();
    w4_ref = w4_str.toFloat();

}
