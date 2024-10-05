#include "walls.h"

bool wallFront() {
    bool frontDetected = (digitalRead(FRONT_SENSOR_PIN) == LOW); // Check for wall in front
    Serial.print("wallFront: ");
    Serial.println(frontDetected);
    return frontDetected;
}

bool wallRight() {
    bool rightDetected = (digitalRead(RIGHT_SENSOR_PIN) == LOW); // Check for wall on the right
    Serial.print("wallRight: ");
    Serial.println(rightDetected);
    return rightDetected;
}

bool wallLeft() {
    bool leftDetected = (digitalRead(LEFT_SENSOR_PIN) == LOW); // Check for wall on the left
    Serial.print("wallLeft: ");
    Serial.println(leftDetected);
    return leftDetected;
}
