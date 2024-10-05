#include "Movement.h"
#include "encoder.h"
// Motor speed variables
int leftSpeed = 145;
int rightSpeed = 145;

// Proportional and derivative gain for feedback control
const float Kp = 5.0;
const float Kd = 0.5;

// Target angle and error tracking
float DesiredAngle = 0.0;
float lastError = 0.0;
/*
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
*/
void moveForward(float distance) {
  //Serial.println("MOVED FORWARD");
     
     float read=0;
     distanceTraveled=0;
     pulseCount=0;
     leftSpeed=0,leftSpeed=0;
  while (read < distance) {
  // get angle
  
     readgyro();
   float angleError = DesiredAngle - anglez;
   float dervative = angleError-lastError;
    // Simple proportional control
    int output = Kp * angleError-Kd * dervative;
     
    output = constrain(output, -255, 255);
    // Control motors
     leftSpeed = 255 - output;
      rightSpeed = 255 + output;

    // Ensure motor speeds are within valid range
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
    //Serial.print(leftSpeed);
    //Serial.print("   ,   ");
    //Serial.println(rightSpeed);

    // Set motor speeds
    analogWrite(MOTOR_RIGHT_PIN1, rightSpeed);
    analogWrite(MOTOR_RIGHT_PIN2, LOW);
    analogWrite(MOTOR_LEFT_PIN1, leftSpeed);
    analogWrite(MOTOR_LEFT_PIN2, LOW);

    delay(10);  // Small delay for stability
    
  lastError=angleError;
  read=EncoderReading();
  }
  //Serial.println("stopppppppppppppp");
  //delay(2000);
  analogWrite(MOTOR_RIGHT_PIN1, 0);
  analogWrite(MOTOR_RIGHT_PIN2, 0);
  analogWrite(MOTOR_LEFT_PIN1, 0);
  analogWrite(MOTOR_LEFT_PIN2, 0);
  delay(500);
  //Serial.println("endddddddddddddd");
}
/*1
// Turn the robot 90 degrees to the right
void turnRight() { 
  Serial.println("TURNED RIGHT");
    DesiredAngle -= 90 * gr; 
    while (fabs(DesiredAngle - anglez) > 5) {// Update target angle for turning right
    readgyro();
    float AngleError = DesiredAngle - anglez;
     float dervative = AngleError-lastError;
    // Simple proportional control
    int output = Kp * AngleError+Kd * dervative;
     // Ensure the adjustment is within valid PWM range
    output = constrain(output, -255, 255);

    // Ensure motor speeds are within valid range
       leftSpeed = constrain(255 - output, 140, 255);
    rightSpeed = constrain(255 + output, 140, 255);
    //Serial.print(leftSpeed);
    //Serial.print("   ,   ");
    //Serial.println(rightSpeed);

    // Set motor speeds
    analogWrite(MOTOR_RIGHT_PIN1, rightSpeed);
    analogWrite(MOTOR_RIGHT_PIN2, LOW);
    analogWrite(MOTOR_LEFT_PIN1, LOW);
    analogWrite(MOTOR_LEFT_PIN2, leftSpeed);

    delay(10);  // Small delay for stability
    
    lastError=AngleError;

  }
pulseCount = 0;
    // Stop motors after turning
    analogWrite(MOTOR_RIGHT_PIN1, LOW);
    analogWrite(MOTOR_RIGHT_PIN2, LOW);
    analogWrite(MOTOR_LEFT_PIN1, LOW);
    analogWrite(MOTOR_LEFT_PIN2, LOW);
    delay(2000);
    anglez=0;
   // DesiredAngle=0;
}*/
// Turn the robot 90 degrees to the right
///////////////////////////////////////////////////////////////////////////////////////
/*2
void turnRight() {
    DesiredAngle -= 90 * gr;  // Update target angle for turning right

    while (fabs(DesiredAngle - anglez) > 5) {
        readgyro(); 
        Serial.println(anglez);

        float angleError = DesiredAngle - anglez;
        float derivative = angleError - lastError;

        int output = Kp * angleError + Kd * derivative;
        output = constrain(output, -255, 255);  // Constrain output to valid range
         //leftSpeed = constrain(255 - output, 50, 255);
       // rightSpeed = constrain(255 + output, 50, 255);
      /////////////////////// output=abs(output);
       
       Serial.print(leftSpeed);
       Serial.print("   ,   ");
       Serial.println(rightSpeed);

    // Set motor speeds
    analogWrite(MOTOR_RIGHT_PIN1, LOW);
    analogWrite(MOTOR_RIGHT_PIN2, rightSpeed);
    analogWrite(MOTOR_LEFT_PIN1, leftSpeed);
    analogWrite(MOTOR_LEFT_PIN2, LOW);

    delay(10);  // Small delay for stability
    
    lastError = angleError;
      
//////////////
        // Set motor speeds
        analogWrite(MOTOR_RIGHT_PIN1,0 );
        analogWrite(MOTOR_RIGHT_PIN2, output);
        analogWrite(MOTOR_LEFT_PIN1, output);
        analogWrite(MOTOR_LEFT_PIN2, 0);

        delay(10);  // Small delay for stability

        lastError = angleError;  // Update last error
        
    }

    // Stop motors after turning
    analogWrite(MOTOR_RIGHT_PIN1, LOW);
    analogWrite(MOTOR_RIGHT_PIN2, LOW);
    analogWrite(MOTOR_LEFT_PIN1, LOW);
    analogWrite(MOTOR_LEFT_PIN2, LOW);
    delay(500);
} 
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void turnRight() {
    DesiredAngle -= 90 * gr;  // Update target angle for turning right
    lastError = 0;  // Initialize lastError for the derivative term

    while (fabs(DesiredAngle - anglez) > 5) {  // Continue until within 5 degrees
        readgyro();  // Update current angle
        Serial.println(anglez);

        float angleError = DesiredAngle - anglez;  // Calculate error
        float derivative = angleError - lastError;  // Calculate derivative of error

        // PID control calculations
        int output = Kp * angleError + Kd * derivative;
        output = constrain(output, -255, 255);  // Constrain output to valid range
 /*
        // Set motor speeds based on output
        leftSpeed = constrain(255 - output, 50, 255);
        rightSpeed = constrain(255 + output, 50, 255);
        /*
        Serial.print(leftSpeed);
        Serial.print("   ,   ");
        Serial.println(rightSpeed);
*/
        // Set motor speeds
        output=abs(output);
         Serial.println("/////");
        Serial.println(output);
        analogWrite(MOTOR_RIGHT_PIN1, LOW);
        analogWrite(MOTOR_RIGHT_PIN2, output);
        analogWrite(MOTOR_LEFT_PIN1, output);
        analogWrite(MOTOR_LEFT_PIN2, LOW);

        delay(10);  // Small delay for stability

        lastError = angleError;  // Update last error for next iteration
    }

    // Stop motors after turning
    analogWrite(MOTOR_RIGHT_PIN1, LOW);
    analogWrite(MOTOR_RIGHT_PIN2, LOW);
    analogWrite(MOTOR_LEFT_PIN1, LOW);
    analogWrite(MOTOR_LEFT_PIN2, LOW);
    delay(500);  // Allow time for the motors to stop completely
}

void turnLeft() {
  Serial.println("TURNED LEFT");
    DesiredAngle += 90 * gr;
    readgyro(); 
    while (fabs(DesiredAngle - anglez) > 5) {// Update target angle for turning right
    readgyro();
    float AngleError = DesiredAngle - anglez;
     float dervative = AngleError-lastError;
    // Simple proportional control
    int output = Kp * AngleError+Kd * dervative;
     // Ensure the adjustment is within valid PWM range
    output = constrain(output, -255, 255);
    output=abs(output);
 

    // Set motor speeds
    analogWrite(MOTOR_RIGHT_PIN1, output);
    analogWrite(MOTOR_RIGHT_PIN2, 0);
    analogWrite(MOTOR_LEFT_PIN1, 0);
    analogWrite(MOTOR_LEFT_PIN2, output);

    delay(10);  // Small delay for stability
    
    lastError=AngleError;

  }
  // Stop motors after turning
    analogWrite(MOTOR_RIGHT_PIN1, LOW);
    analogWrite(MOTOR_RIGHT_PIN2, LOW);
    analogWrite(MOTOR_LEFT_PIN1, LOW);
    analogWrite(MOTOR_LEFT_PIN2, LOW);
    delay(500);
    //anglez=0;
    //DesiredAngle=0;
} 
/////////////////////////////////
/*
void turnLeft() {
  Serial.println("TURNED LEFT");
    DesiredAngle += 90 * gr;
    readgyro(); 
    while (fabs(DesiredAngle - anglez) > 5) {// Update target angle for turning right
    readgyro();
    float AngleError = DesiredAngle - anglez;
     float dervative = AngleError-lastError;
    // Simple proportional control
    int output = Kp * AngleError+Kd * dervative;
     // Ensure the adjustment is within valid PWM range
    output = constrain(output, -255, 255);
    output=abs(output);
 

    // Set motor speeds
    analogWrite(MOTOR_RIGHT_PIN1, output);
    analogWrite(MOTOR_RIGHT_PIN2, 0);
    analogWrite(MOTOR_LEFT_PIN1, 0);
    analogWrite(MOTOR_LEFT_PIN2, output);

    delay(10);  // Small delay for stability
    
    lastError=AngleError;

  }
  // Stop motors after turning
    analogWrite(MOTOR_RIGHT_PIN1, LOW);
    analogWrite(MOTOR_RIGHT_PIN2, LOW);
    analogWrite(MOTOR_LEFT_PIN1, LOW);
    analogWrite(MOTOR_LEFT_PIN2, LOW);
    delay(2000);
    //anglez=0;
    //DesiredAngle=0;
}
*/

