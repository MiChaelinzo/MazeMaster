#include <Arduino.h>
#include <SharpIR.h>
#include <L298N.h>

// Pin Definitions (Adjust to your wiring)
#define IR_FRONT_PIN A0
#define IR_LEFT_PIN A1
#define IR_RIGHT_PIN A2
#define MOTOR_LEFT_PWM 9
#define MOTOR_LEFT_DIR 8
#define MOTOR_RIGHT_PWM 10
#define MOTOR_RIGHT_DIR 11

// Sensor Objects
SharpIR sharpIRFront(IR_FRONT_PIN, 1080);
SharpIR sharpIRLeft(IR_LEFT_PIN, 1080);
SharpIR sharpIRRight(IR_RIGHT_PIN, 1080);

// Motor Objects
L298N motorLeft(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR);
L298N motorRight(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR);

// PID Constants (Tune these!)
const float Kp = 0.5; // Proportional gain
const float Ki = 0.1; // Integral gain
const float Kd = 0.05; // Derivative gain

// Other Constants
const int BASE_SPEED = 150;
const int DESIRED_DISTANCE = 15; // cm
const int WALL_FOLLOWING_DISTANCE = 10; // cm

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


void determineNextMove() {
    // Simple wall-following logic
    if (sharpIRRight.distance() < WALL_FOLLOWING_DISTANCE) {
        turnLeft(); 
    } else if (sharpIRFront.distance() < DESIRED_DISTANCE) {
        turnRight();
    } else {
        moveForward();
    }
}

void executeMovement() {
    // PID Control for wall following
    int error = DESIRED_DISTANCE - sharpIRLeft.distance();
    integral += error;
    int derivative = error - prevError;
    prevError = error;

    int motorSpeedAdjustment = Kp * error + Ki * integral + Kd * derivative;

    int leftSpeed = BASE_SPEED + motorSpeedAdjustment;
    int rightSpeed = BASE_SPEED - motorSpeedAdjustment;

    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    motorLeft.setSpeed(leftSpeed);
    motorRight.setSpeed(rightSpeed);

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
