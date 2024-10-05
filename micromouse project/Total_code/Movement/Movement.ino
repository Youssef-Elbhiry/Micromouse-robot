#include "Movement.h"

// Motor speed variables
int leftSpeed = 145;
int rightSpeed = 145;

// Proportional and derivative gain for feedback control
const float Kp = 5.0;
const float Kd = 0.5;

// Target angle and error tracking
float DesiredAngle = 0.0;
float lastError = 0.0;

// Move the robot forward by a specified distance
void moveForward(float distance) {
    DesiredAngle = 0.0;  // Target angle is straight (0 degrees)

    while (EncoderReading() < distance) {
        readgyro();  // Get the current gyroscope reading

        float angleError = DesiredAngle - anglez;  // Calculate angle error
        float derivative = angleError - lastError; // Calculate error derivative

        // Simple proportional and derivative control
        int output = Kp * angleError - Kd * derivative;
        output = constrain(output, -255, 255);  // Constrain output to valid range

        // Adjust motor speeds based on output
        leftSpeed = constrain(255 - output, 0, 255);
        rightSpeed = constrain(255 + output, 0, 255);

        // Set motor speeds
        analogWrite(MOTOR_RIGHT_PIN1, rightSpeed);
        analogWrite(MOTOR_RIGHT_PIN2, LOW);
        analogWrite(MOTOR_LEFT_PIN1, leftSpeed);
        analogWrite(MOTOR_LEFT_PIN2, LOW);

        delay(10);  // Small delay for stability

        lastError = angleError;  // Update last error
    }

    // Stop motors after moving
    digitalWrite(MOTOR_RIGHT_PIN1, LOW);
    digitalWrite(MOTOR_RIGHT_PIN2, LOW);
    digitalWrite(MOTOR_LEFT_PIN1, LOW);
    digitalWrite(MOTOR_LEFT_PIN2, LOW);
}

// Turn the robot 90 degrees to the right
void turnRight() {
    DesiredAngle -= 90 * gr;  // Update target angle for turning right

    while (fabs(DesiredAngle - anglez) > 5) {
        readgyro();  // Get the current gyroscope reading

        float angleError = DesiredAngle - anglez;
        float derivative = angleError - lastError;

        // Simple proportional and derivative control
        int output = Kp * angleError + Kd * derivative;
        output = constrain(output, -255, 255);  // Constrain output to valid range

        // Adjust motor speeds based on output
        leftSpeed = constrain(255 - output, 0, 255);
        rightSpeed = constrain(255 + output, 0, 255);

        // Set motor speeds
        analogWrite(MOTOR_RIGHT_PIN1, rightSpeed);
        analogWrite(MOTOR_RIGHT_PIN2, LOW);
        analogWrite(MOTOR_LEFT_PIN1, leftSpeed);
        analogWrite(MOTOR_LEFT_PIN2, LOW);

        delay(10);  // Small delay for stability

        lastError = angleError;  // Update last error
    }

    // Stop motors after turning
    analogWrite(MOTOR_RIGHT_PIN1, LOW);
    analogWrite(MOTOR_RIGHT_PIN2, LOW);
    analogWrite(MOTOR_LEFT_PIN1, LOW);
    analogWrite(MOTOR_LEFT_PIN2, LOW);
}

// Turn the robot 90 degrees to the left
void turnLeft() {
    DesiredAngle += 90 * gr;  // Update target angle for turning left

    while (fabs(DesiredAngle - anglez) > 5) {
        readgyro();  // Get the current gyroscope reading

        float angleError = DesiredAngle - anglez;
        float derivative = angleError - lastError;

        // Simple proportional and derivative control
        int output = Kp * angleError + Kd * derivative;
        output = constrain(output, -255, 255);  // Constrain output to valid range

        // Adjust motor speeds based on output
        leftSpeed = constrain(255 + output, 0, 255);
        rightSpeed = constrain(255 - output, 0, 255);

        // Set motor speeds
        analogWrite(MOTOR_RIGHT_PIN1, rightSpeed);
        analogWrite(MOTOR_RIGHT_PIN2, LOW);
        analogWrite(MOTOR_LEFT_PIN1, leftSpeed);
        analogWrite(MOTOR_LEFT_PIN2, LOW);

        delay(10);  // Small delay for stability

        lastError = angleError;  // Update last error
    }

    // Stop motors after turning
    analogWrite(MOTOR_RIGHT_PIN1, LOW);
    analogWrite(MOTOR_RIGHT_PIN2, LOW);
    analogWrite(MOTOR_LEFT_PIN1, LOW);
    analogWrite(MOTOR_LEFT_PIN2, LOW);
}
