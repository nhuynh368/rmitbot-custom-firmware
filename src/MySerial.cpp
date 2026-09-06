#include "MySerial.h"

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
    while (!Serial)
        ;
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
    int tabIndex = msg.indexOf('\t');
    
    if (tabIndex >= 2)
    {
        String w1_str = msg.substring(0, tabIndex);
        String w2_str = msg.substring(tabIndex + 1, 2 * tabIndex);
        String w3_str = msg.substring(2 * tabIndex + 1, 3 * tabIndex);
        String w4_str = msg.substring(3 * tabIndex + 1);

        w1_ref = w1_str.toFloat();
        w2_ref = w2_str.toFloat();
        w3_ref = w3_str.toFloat();
        w4_ref = w4_str.toFloat();
    }
}
