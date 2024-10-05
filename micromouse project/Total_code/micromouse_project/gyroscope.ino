#include "gyroscope.h"

// Variable to store the gyro offset
float offset3 = 0;

void initgyro() {
    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10); // Wait indefinitely if the MPU6050 is not found
        }
    }
    Serial.println("MPU6050 Found!");

    // Configure MPU6050 settings
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Calculate gyro offset
    for (int i = 0; i < 200; i++) {
        mpu.getEvent(&a, &g, &temp);
        offset3 += g.gyro.z; // Accumulate gyro z-axis readings
    }
    offset3 /= 200.0; // Average the readings
}

void readgyro() {
    float time = millis();
    float dt = (time - lasttime) / 1000.0; // Time interval in seconds

    // Get sensor readings
    mpu.getEvent(&a, &g, &temp);

    // Calculate the change in angle
    float area = (g.gyro.z - offset3) * dt * 180 / M_PI;
    if (fabs(area) > 0.007) {
        anglez += area; // Update angle if the change is significant
    }

    lasttime = time; // Update the last time for the next reading

    // Print the current angle for debugging
    //Serial.print("anglez = ");
    //Serial.println(anglez);
}
