#ifndef MOVEMENT_FUNCTIONS_H
#define MOVEMENT_FUNCTIONS_H

#include <math.h>
#include "encoder.h"
#include "gyroscope.h"

// Motor control pin definitions
#define MOTOR_RIGHT_PIN1 5
#define MOTOR_RIGHT_PIN2 18
#define MOTOR_LEFT_PIN1 4
#define MOTOR_LEFT_PIN2 0

// Function declarations
void moveForward(float distance);
void turnLeft();
void turnRight();

#endif // MOVEMENT_FUNCTIONS_H
