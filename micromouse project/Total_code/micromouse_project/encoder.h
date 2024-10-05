#ifndef ENCODER_FUNCTIONS_H
#define ENCODER_FUNCTIONS_H
extern volatile long pulseCount ;
void EncoderSetup();
float EncoderReading();
float distanceTraveled;
#endif // ENCODER_FUNCTIONS_H
