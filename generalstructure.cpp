#include <Arduino.h>
#include <SharpIR.h> // Example IR sensor library
#include <HCSR04.h> // Example ultrasonic sensor library
#include <L298N.h>  // Example motor driver library

// Pin Definitions
#define IR_FRONT_PIN A0
#define IR_LEFT_PIN A1
#define IR_RIGHT_PIN A2
#define TRIGGER_PIN 7
#define ECHO_PIN 8
#define MOTOR_LEFT_1 2
#define MOTOR_LEFT_2 3
#define MOTOR_RIGHT_1 4
#define MOTOR_RIGHT_2 5

// Sensor Objects
SharpIR sharpIRFront(IR_FRONT_PIN, 1080); // Model GP2Y0A21YK0F
UltraSonicDistanceSensor distanceSensor(TRIGGER_PIN, ECHO_PIN);

// Motor Objects
L298N motorLeft(MOTOR_LEFT_1, MOTOR_LEFT_2);
L298N motorRight(MOTOR_RIGHT_1, MOTOR_RIGHT_2);

// Global Variables
int frontDistance, leftDistance, rightDistance;  // IR sensor distances
long ultrasonicDistance; // Ultrasonic sensor distance

const int MAZE_SIZE = 16; // Example maze size (16x16)
bool mazeMap[MAZE_SIZE][MAZE_SIZE];
int robotX, robotY;
int robotDirection; // 0 = North, 1 = East, 2 = South, 3 = West

void setup() {
    Serial.begin(9600);
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            mazeMap[i][j] = false; // Initialize maze map
        }
    }
    robotX = 0;
    robotY = 0;
    robotDirection = 0; // Start facing North
}

void loop() {
    readSensorData();
    updateMazeMap();
    determineNextMove();
    executeMovement();
    delay(100); // Adjust for smoother movement
}

void readSensorData() {
    frontDistance = sharpIRFront.distance();
    leftDistance = sharpIRLeft.distance();
    rightDistance = sharpIRRight.distance();

    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN,   
 LOW);
    ultrasonicDistance = microsecondsToCentimeters(pulseIn(ECHO_PIN, HIGH));
}

void updateMazeMap() {
    // Use sensor data to update maze map
    // ... (Implementation depends on the specific maze representation you choose)

    // Example for a simple grid-based map
    int currentCellX = robotX / CELL_SIZE;
    int currentCellY = robotY / CELL_SIZE;

    if (frontDistance < WALL_THRESHOLD) {
        mazeMap[currentCellX][currentCellY + 1] = 1; // Wall in front
    }
    if (leftDistance < WALL_THRESHOLD) {
        mazeMap[currentCellX - 1][currentCellY] = 1; // Wall on the left
    }
    if (rightDistance < WALL_THRESHOLD) {
        mazeMap[currentCellX + 1][currentCellY] = 1; // Wall on the right
    }
}

void determineNextMove() {
    // ... (Maze-solving algorithm implementation)

    // Example: Simple left-hand wall follower
    if (leftDistance > CLEAR_PATH_THRESHOLD) {
        turnLeft();
    } else if (frontDistance < WALL_THRESHOLD) {
        turnRight();
    } else {
        moveForward();
    }
}

void executeMovement() {
    // ... (Motor control based on the decided move)

    // Example: Move forward
    motorLeft.setSpeed(200); // Adjust speed as needed
    motorRight.setSpeed(200);
    motorLeft.forward();
    motorRight.forward();
}

// Helper functions for movement
void moveForward() {
    motorLeft.setSpeed(BASE_SPEED);
    motorRight.setSpeed(BASE_SPEED);
    motorLeft.forward();
    motorRight.forward();
}

void moveBackward() {
    motorLeft.setSpeed(BASE_SPEED);
    motorRight.setSpeed(BASE_SPEED);
    motorLeft.backward();
    motorRight.backward();
}

void turnLeft() {
    motorLeft.setSpeed(TURN_SPEED); // Slower for precise turning
    motorRight.setSpeed(TURN_SPEED);
    motorLeft.backward();
    motorRight.forward();
    delay(TURN_DELAY); // Adjust delay for desired turn angle
    motorLeft.stop();
    motorRight.stop();
}

void turnRight() {
    motorLeft.setSpeed(TURN_SPEED);
    motorRight.setSpeed(TURN_SPEED);
    motorLeft.forward();
    motorRight.backward();
    delay(TURN_DELAY);
    motorLeft.stop();
    motorRight.stop();
}

void stopMotors() {
    motorLeft.stop();
    motorRight.stop();
}

// executeMovement function (with added helper function calls)
void executeMovement() {
    switch (nextMove) { // Assuming 'nextMove' is determined in determineNextMove()
        case MOVE_FORWARD:
            moveForward();
            break;
        case MOVE_BACKWARD:
            moveBackward();
            break;
        case TURN_LEFT:
            turnLeft();
            break;
        case TURN_RIGHT:
            turnRight();
            break;
        case STOP:
            stopMotors();
            break;
    }
}
