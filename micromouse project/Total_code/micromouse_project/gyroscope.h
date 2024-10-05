#ifndef GYROSCOPE_FUNCTIONS_H
#define GYROSCOPE_FUNCTIONS_H

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// External variables
extern float anglez;
extern float lasttime;

// Constants
const float gr = 88.41 / 90.0;

// MPU6050 object and sensor event variables
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

// Function prototypes
void initgyro();
void readgyro();

#endif // GYROSCOPE_FUNCTIONS_H
