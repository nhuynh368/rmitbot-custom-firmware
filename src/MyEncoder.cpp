#include "MyEncoder.h"

// ==============================================
// Constructor for the Encoder class
// ==============================================
Encoder::Encoder(byte ENCA, byte ENCB) // Constructor for the Encoder class
{
    _ENCA = ENCA; // From public ENC_A to private _ENC_A
    _ENCB = ENCB; // From public ENC_B to private _ENC_B
    w_time = 0;
    th_prev = 0;
    w = 0;
}

void Encoder::begin() // Initialize the encoder
{
    _encoder.setFilter(1023);
    _encoder.attachFullQuad(_ENCA, _ENCB); // Attach the encoder to the specified pins
    _encoder.setCount(0);                  // Set the initial count to 0
    w_time = micros();
}

long Encoder::getCount() // Get the current count from the encoder
{
    return _encoder.getCount();
}

double Encoder::getVelocity() // Get the velocity of the encoder
{
    th = _encoder.getCount() * 2 * PI / _ENC_RES; // motor angular position in radians
    
    unsigned long currentTime = micros();
    unsigned long dt_us = currentTime - w_time;

    if (dt_us >= 100000) 
    {
        double dt_sec = dt_us * 1e-6;

        if (dt_sec > 0.0)
        {
            w_raw = (th - th_prev) / dt_sec; // Calculate the unfiltered velocity
            w = alpha * w_raw + (1 - alpha) * w; // Calculate the filtered velocity
        }

        w_time = currentTime;
        th_prev = th;
    }
    return w;
    
    //if (micros() - w_time >= 100 * 1e3)           // Velocity is calculated every 100ms
    //{
    //    w_raw = (th - th_prev) / ((micros() - w_time) * 1e-6); // Calculate the unfiltered velocity
    //   w = alpha * w_raw + (1 - alpha) * w;                   // Calculate the filtered velocity
    //    w_time = micros();
    //    th_prev = th;
    //}
    //return w;
}