#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>

// --- Pin Definitions ---
#define TRIGGER_PIN_1 7
#define ECHO_PIN_1 6
#define TRIGGER_PIN_2 5
#define ECHO_PIN_2 4
#define LEG1_SERVO_PIN 9
#define LEG2_SERVO_PIN 10
#define LEG3_SERVO_PIN 11
#define LEG4_SERVO_PIN 12
#define BATTERY_PIN A3      
#define LED_BUILTIN 13

// --- Sensor Objects ---
NewPing sonarFront(TRIGGER_PIN_1, ECHO_PIN_1, 200); 
NewPing sonarRight(TRIGGER_PIN_2, ECHO_PIN_2, 200); 

// --- Servo Motor Objects ---
Servo legServos[4];

// --- Robot State ---
struct Robot {
    int x;
    int y;
    int direction; // 0 = North, 1 = East, 2 = South, 3 = West
} robot;

// --- Gait Control ---
struct Gait {
    int phases;
    int sequence[4][4]; 
    int delays[4];       
} tripodGait = {
    4, 
    {
        {1, 0, 1, 0},
        {0, 1, 0, 1},
        {-1, 0, -1, 0},
        {0, -1, 0, -1}
    },
    {200, 200, 200, 200}
};

// --- Maze Representation ---
const int MAZE_SIZE = 16;
bool mazeMap[MAZE_SIZE][MAZE_SIZE];

// --- Movement Constants ---
const int STEP_ANGLE = 15;
const int MIN_LEG_ANGLE = 45;
const int MAX_LEG_ANGLE = 135;
const int TURN_ANGLE_INCREMENT = 10;
const int TURN_DELAY_INCREMENT = 50;
const float VOLTAGE_DIVIDER_RATIO = 2.0; 
const float BATTERY_LOW_THRESHOLD = 6.0; 

// --- Global Variables ---
int legAngles[4] = {90, 90, 90, 90}; 
int gaitPhase = 0;
unsigned int distanceFront, distanceRight;

// --- Function Prototypes ---
void readSensorData();
void updateMazeMap();
void determineNextMove();
void executeMovement();
void moveForward();
void turnRight();
void turnLeft();
void stopMotors();
bool checkForErrors();
void handleError();
void updateServoPositions();

// ======================================================
// Setup
// ======================================================
void setup() {
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);

    // Attach servo motors 
    for (int i = 0; i < 4; i++) {
        legServos[i].attach(LEG1_SERVO_PIN + i); 
    }

    // Initialize robot state
    robot.x = 0;
    robot.y = 0;
    robot.direction = 0; 

    // Initialize maze map
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            mazeMap[i][j] = false; 
        }
    }
}

// ======================================================
// Main Loop 
// ======================================================
void loop() {
    if (checkForErrors()) {
        handleError();
        return; 
    }

    readSensorData();
    updateMazeMap();
    determineNextMove();
    executeMovement();
}

// ======================================================
// Functions
// ======================================================

void readSensorData() {
    distanceFront = sonarFront.ping_cm();
    distanceRight = sonarRight.ping_cm();
}

void updateMazeMap() {
    // Update mazeMap based on sensor data and robot position/direction
    mazeMap[robot.x][robot.y] = true; // Mark current cell as visited

    // Example using right sensor:
    if (distanceRight < 10) { // Adjust threshold as needed
        switch (robot.direction) {
            case 0: // North
                mazeMap[robot.x + 1][robot.y] = true; // Wall to the east
                break;
            case 1: // East
                mazeMap[robot.x][robot.y - 1] = true; // Wall to the south
                break;
            case 2: // South
                mazeMap[robot.x - 1][robot.y] = true; // Wall to the west
                break;
            case 3: // West
                mazeMap[robot.x][robot.y + 1] = true; // Wall to the north
                break;
        }
    }

    // Add logic for front and other sensors to detect walls 
}


void determineNextMove() {
    // Example: Right-hand wall following
    if (distanceRight > 15) { 
        turnRight();
    } else if (distanceFront < 10) { 
        turnLeft();
    } else { 
        moveForward();
    }
}

void executeMovement() {
    // Tripod Gait
    for (int leg = 0; leg < 4; leg++) {
        legAngles[leg] += STEP_ANGLE * tripodGait.sequence[gaitPhase][leg];
        legAngles[leg] = constrain(legAngles[leg], MIN_LEG_ANGLE, MAX_LEG_ANGLE);
        legServos[leg].write(legAngles[leg]);
    }

    gaitPhase = (gaitPhase + 1) % tripodGait.phases;
    delay(tripodGait.delays[gaitPhase]);
}

void moveForward() {
    executeMovement();

    switch (robot.direction) {
        case 0: // North
            robot.y++;
            break;
        case 1: // East
            robot.x++;
            break;
        case 2: // South
            robot.y--;
            break;
        case 3: // West
            robot.x--;
            break;
    }
}

void turnRight() {
    // 1. Lift and shift legs for turning
    legAngles[0] += 30; 
    legAngles[3] -= 30;  
    legAngles[1] -= 15; 
    legAngles[2] += 15;  
    updateServoPositions();
    delay(200);

    // 2. Rotate the body (adjust angles as needed)
    for (int i = 0; i < 90 / TURN_ANGLE_INCREMENT; i++) { 
        for (int leg = 0; leg < 4; leg++) {
            legAngles[leg] += TURN_ANGLE_INCREMENT;
        }
        updateServoPositions();
        delay(TURN_DELAY_INCREMENT);
    }

    // 3. Return legs to neutral
    for (int i = 0; i < 4; i++) {
        legAngles[i] = 90;
    }
    updateServoPositions();

    robot.direction = (robot.direction + 1) % 4; 
}

void turnLeft() {
    legAngles[1] += 30; 
    legAngles[2] -= 30;
    legAngles[0] -= 15; 
    legAngles[3] += 15;  
    updateServoPositions();
    delay(200);

    for (int i = 0; i < 90 / TURN_ANGLE_INCREMENT; i++) {
        for (int leg = 0; leg < 4; leg++) {
            legAngles[leg] -= TURN_ANGLE_INCREMENT; 
        }
        updateServoPositions();
        delay(TURN_DELAY_INCREMENT);
    }

    for (int i = 0; i < 4; i++) {
        legAngles[i] = 90;
    }
    updateServoPositions();

    robot.direction = (robot.direction - 1 + 4) % 4; 
}


bool checkForErrors() {
    if (sonarFront.ping_cm() == 0 || sonarRight.ping_cm() == 0) {
        Serial.println("Error: Ultrasonic sensor failure");
        return true;
    }

    int batteryVoltage = analogRead(BATTERY_PIN);
    float voltage = batteryVoltage * (5.0 / 1023.0) * VOLTAGE_DIVIDER_RATIO;
    if (voltage < BATTERY_LOW_THRESHOLD) {
        Serial.println("Error: Low battery");
        return true;
    }

    return false; 
}

void handleError() {
    stopMotors();
    while (true) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100); 
        digitalWrite(LED_BUILTIN, LOW);
        delay(100); 
    }
}

void updateServoPositions() {
    for (int i = 0; i < 4; i++) {
        legAngles[i] = constrain(legAngles[i], MIN_LEG_ANGLE, MAX_LEG_ANGLE);
        legServos[i].write(legAngles[i]);
    }
}

void stopMotors() {
    for (int i = 0; i < 4; i++) {
        legAngles[i] = 90; // Set to neutral
        legServos[i].write(legAngles[i]);
    }
}
