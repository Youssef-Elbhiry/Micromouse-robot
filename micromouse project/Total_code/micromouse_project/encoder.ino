#include "encoder.h"

// Constants for the encoder
const int encoderPinB = 19; // Encoder output B
const int PPR = 36; // Pulses per revolution
const float wheelDiameter = 3.5; // Diameter of the wheel in cm
const float wheelCircumference = 3.14159 * wheelDiameter; // Circumference of the wheel
const float distancePerPulse = wheelCircumference / PPR;
// Variables to store pulse counts
extern  volatile long pulseCount = 0;

// Interrupt service routine for encoder
void ISR_encoder() {
    pulseCount++;
}

// Setup function for the encoder
void EncoderSetup() {
    // Initialize serial communication
    // Serial.begin(115200);

    // Set encoder pin as input
    pinMode(encoderPinB, INPUT);

    // Attach interrupt to encoder pin
    attachInterrupt(digitalPinToInterrupt(encoderPinB), ISR_encoder, CHANGE);
}
 
// Read the encoder data and calculate distance traveled
float EncoderReading() {
    // Detach interrupt to prevent counting pulses during calculation
    detachInterrupt(digitalPinToInterrupt(encoderPinB));

    // Print pulse count for debugging
    //Serial.print("Pulse Count: ");
    //Serial.println(pulseCount);

    // Calculate distance traveled
   
    distanceTraveled= pulseCount * distancePerPulse;

    // Print the distance traveled
    //Serial.print("Distance Traveled: ");
    //Serial.print(distanceTraveled);
    //Serial.println(" cm");

    // Reattach interrupt for future pulse counting
    attachInterrupt(digitalPinToInterrupt(encoderPinB), ISR_encoder, CHANGE);

    // Add a small delay to avoid flooding the serial monitor
    
        //Serial.print("Pulse Count: ");
    //Serial.println(pulseCount);
// Print the distance traveled
    //Serial.print("Distance Traveled: ");
   // Serial.print(distanceTraveled);
   // Serial.println(" cm");
    return distanceTraveled;
}
