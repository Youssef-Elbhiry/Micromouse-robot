#include <vector>
#include "encoder.h"
#include "gyroscope.h"
#include "algo.h"
#include "Movement.h"
#include "walls.h"

// Algorithm variables
short direction = NORTH;
short robot_x =15;
short robot_y = 0;

float anglez = 0.0f, lasttime = 0.0f, dt = 0.0f;

std::vector<std::pair<int, int>> centerPoints = {
    {MAZE_SIZE / 2 - 1, MAZE_SIZE / 2 - 1},
    {MAZE_SIZE / 2 - 1, MAZE_SIZE / 2},
    {MAZE_SIZE / 2, MAZE_SIZE / 2 - 1},
    {MAZE_SIZE / 2, MAZE_SIZE / 2}
};

// Pin mode configuration for sensors and motors
void setupPinModes() {
    // IR sensors
    pinMode(FRONT_SENSOR_PIN, INPUT);
    pinMode(RIGHT_SENSOR_PIN, INPUT);
    pinMode(LEFT_SENSOR_PIN, INPUT);

    // Motor control pins
    pinMode(MOTOR_RIGHT_PIN1, OUTPUT);
    pinMode(MOTOR_RIGHT_PIN2, OUTPUT);
    pinMode(MOTOR_LEFT_PIN1, OUTPUT);
    pinMode(MOTOR_LEFT_PIN2, OUTPUT);
}

// Encoder setup
void encoderSetup() {
    // Attach encoder interrupt pins (example commented out)
    // attachInterrupt(digitalPinToInterrupt(encoderPinA), ISR_encoder, RISING);
}

// Setup function
void setup() {
    Serial.begin(115200);

    // Initialize pin modes, encoder, and gyroscope
    setupPinModes();
    EncoderSetup();
    initgyro();

    anglez = 0.0f;
    lasttime = millis();

    // Perform BFS to initialize the maze exploration
    bfs(centerPoints);
    for(int i=0;i<MAZE_SIZE;i++)
        {
         for(int j=0;j<MAZE_SIZE;j++) 
         {
             Serial.print(maze[i][j]);
             Serial.print(" ");
         }
          Serial.print("\n");
        }
    
}

// Main loop function
void loop() {
    // Continue navigating the maze until the robot reaches the goal
   
    while (maze[robot_x][robot_y] != 0) {
       // Serial.println("do not get to the goal ");
        updateWalls(robot_x, robot_y, direction);
       // Serial.println("updated the walls done ");

        bfs(centerPoints);

        Serial.print("in loop robot_x =");
        Serial.print(robot_x);
       Serial.print(" ,robot_y=");
        Serial.print(robot_y);

        moveToBestCell(robot_x, robot_y, direction);
    } 
   

    delay(1000); // Short delay for stability
}

