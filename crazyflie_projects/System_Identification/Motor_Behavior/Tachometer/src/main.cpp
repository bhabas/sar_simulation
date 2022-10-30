#include <Arduino.h>

// DEFINE VARIABLES
double omega;                   // Rotation rate[rad/s]
char strBuf[25];                // Print buffer 
unsigned long t_prev = 0;       // time [ms]
unsigned long t_delta;          // [ms]
unsigned long sample_rate = 50; // [ms]
volatile unsigned int count;    // Detection count



// INTERRUPT SERVICE ROUTINE
void isr()
{
    count++;
}

void setup() {

    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(2),isr,FALLING);
}

void loop() {

    t_delta = millis() - t_prev;

    if(t_delta > sample_rate)
    {
        // CALC ROTATION RATE
        omega = M_PI*count/(t_delta*1e-3); // [rad/s]
        Serial.println(String(millis()) + ", " + String(omega));

        // RESET VALUES
        count = 0;
        t_prev = millis();
    }
}

