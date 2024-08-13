#include <Arduino.h>
#include <Servo.h> 
#include <NewPing.h> 

// Pin Definitions
#define TRIGGER_PIN_1 7
#define ECHO_PIN_1 6
#define TRIGGER_PIN_2 5
#define ECHO_PIN_2 4
#define LEG1_SERVO_PIN 9  // Adjust as needed
#define LEG2_SERVO_PIN 10 // Adjust as needed
#define LEG3_SERVO_PIN 11 // Adjust as needed
#define LEG4_SERVO_PIN 12 // Adjust as needed

// Sensor Objects
NewPing sonarFront(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE); 
NewPing sonarRight(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE); 

// Servo Motor Objects
Servo leg1Servo;
Servo leg2Servo;
Servo leg3Servo;
Servo leg4Servo;

// Global Variables
const int MAZE_SIZE = 16; // Example maze size (16x16)
bool mazeMap[MAZE_SIZE][MAZE_SIZE];
int robotX, robotY;
int robotDirection; // 0 = North, 1 = East, 2 = South, 3 = West

int legAngles[4]; 
int gaitPhase = 0; 

// Movement constants
const int STEP_ANGLE = 15;  // Adjust as needed
const int MIN_LEG_ANGLE = 45; // Adjust as needed
const int MAX_LEG_ANGLE = 135; // Adjust as needed
const int GAIT_DELAY = 200;  // Adjust as needed

void setup() {
    Serial.begin(9600);

    // Attach servo motors to pins
    leg1Servo.attach(LEG1_SERVO_PIN);
    leg2Servo.attach(LEG2_SERVO_PIN);
    leg3Servo.attach(LEG3_SERVO_PIN);
    leg4Servo.attach(LEG4_SERVO_PIN);

    // Initialize leg angles to a neutral position
    for (int i = 0; i < 4; i++) {
        legAngles[i] = 90; 
    }

    // Initialize maze map
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
    // Check for errors and handle them before proceeding
    if (checkForErrors()) {
        handleError();
        return; 
    }

    readSensorData();
    updateMazeMap();
    determineNextMove();
    executeMovement();
}

void readSensorData() {
    // Read ultrasonic sensors
    unsigned int distanceFront = sonarFront.ping_cm();
    unsigned int distanceRight = sonarRight.ping_cm();
    // ... (Read other sensors if needed)

    // You'll likely want to store these distances in global variables or use them directly in determineNextMove()
}

void updateMazeMap() {
    // Update the mazeMap array based on sensor data
    // ... (Implementation depends on the specific maze representation you choose)
}

void determineNextMove() {
    // Implement your maze-solving algorithm here (e.g., Wall Follower, Flood Fill, etc.)
    // ... 

    // Example: Simple wall following (using right-hand rule)
    if (distanceRight > 15) { // If there's space on the right, turn right
        turnRight();
    } else if (distanceFront < 10) { // If there's a wall in front, turn left
        turnLeft();
    } else { // Otherwise, move forward
        moveForward();
    }
}

// ... (updateMazeMap and determineNextMove functions - implementation depends on your chosen maze-solving algorithm)

void executeMovement() {
    // Tripod gait example
    switch (gaitPhase) {
        case 0:
            // Move legs 1 and 3 forward
            legAngles[0] += STEP_ANGLE;
            legAngles[2] += STEP_ANGLE;
            break;
        case 1:
            // Move legs 2 and 4 forward
            legAngles[1] += STEP_ANGLE;
            legAngles[3] += STEP_ANGLE;
            break;
        case 2:
            // Move legs 1 and 3 back
            legAngles[0] -= STEP_ANGLE;
            legAngles[2] -= STEP_ANGLE;
            break;
        case 3:
            // Move legs 2 and 4 back
            legAngles[1] -= STEP_ANGLE;
            legAngles[3] -= STEP_ANGLE;
            break;
    }

    // Update servo positions
    for (int i = 0; i < 4; i++) {
        legAngles[i] = constrain(legAngles[i], MIN_LEG_ANGLE, MAX_LEG_ANGLE); // Ensure angles are within limits
        switch (i) {
            case 0:
                leg1Servo.write(legAngles[i]);
                break;
            case 1:
                leg2Servo.write(legAngles[i]);
                break;
            case 2:
                leg3Servo.write(legAngles[i]);
                break;
            case 3:
                leg4Servo.write(legAngles[i]);
                break;
        }
    }

    // Advance to the next gait phase
    gaitPhase = (gaitPhase + 1) % 4;

    delay(GAIT_DELAY); // Adjust for desired gait speed
}

// Helper functions for turning, error handling, etc.
void turnRight() {
    // Implementation for turning the robot right
    // This will depend on your specific leg configuration and mechanics
    // You might need to experiment to find the best sequence of leg movements
    // Here's a more elaborate example:

    // 1. Lift and move legs to initiate the turn
    legAngles[0] += 30; // Lift front right leg
    legAngles[3] -= 30; // Lower back left leg
    legAngles[1] -= 15; // Slightly retract front left leg
    legAngles[2] += 15; // Slightly extend back right leg
    updateServoPositions(); // Update servo positions
    delay(200); // Short pause

    // 2. Rotate the body clockwise (adjust angles and delays as needed)
    for (int i = 0; i < 4; i++) {
        legAngles[i] += TURN_ANGLE_INCREMENT; // Adjust the increment for smoother turning
        // ... (Update servo positions)
        delay(TURN_DELAY_INCREMENT);
    }

    // 3. Return legs to neutral position
    for (int i = 0; i < 4; i++) {
        legAngles[i] = 90;
        // ... (Update servo positions)
    }
}

void turnLeft() {
    // Implementation for turning the robot left
    // Similar to turnRight, but with adjustments for counter-clockwise rotation
    // ... (Follow a similar pattern as turnRight, but adjust leg movements and angle increments accordingly)
}

bool checkForErrors() {
    // Check for sensor errors (invalid or out-of-range readings)
    if (sonarFront.ping_cm() == 0 || sonarRight.ping_cm() == 0) {
        Serial.println("Error: Ultrasonic sensor failure");
        return true;
    }

    // Check for motor stalls (this might require additional hardware or current sensing)
    // You could monitor motor current or encoder feedback to detect stalls
    // ... 

    // Check for low battery (you'll need a voltage divider circuit and analogRead)
    int batteryVoltage = analogRead(BATTERY_PIN); 
    float voltage = batteryVoltage * (5.0 / 1023.0) * VOLTAGE_DIVIDER_RATIO; // Adjust for your voltage divider
    if (voltage < BATTERY_LOW_THRESHOLD) {
        Serial.println("Error: Low battery");
        return true;
    }

    return false; // No errors detected
}

void handleError() {
    // Stop all motors
    for (int i = 0; i < 4; i++) {
        legAngles[i] = 90; // Neutral position
        // ... (Update servo positions)
    }

    // Signal the error (e.g., blink an LED or send a message via serial)
    while (true) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250); // Faster blink for error indication
        digitalWrite(LED_BUILTIN, LOW);
        delay(250);
    }
}

void updateServoPositions() {
    for (int i = 0; i < 4; i++) {
        legAngles[i] = constrain(legAngles[i], MIN_LEG_ANGLE, MAX_LEG_ANGLE);
        switch (i) {
            case 0:
                leg1Servo.write(legAngles[i]);
                break;
            case 1:
                leg2Servo.write(legAngles[i]);
                break;
            case 2:
                leg3Servo.write(legAngles[i]);
                break;
            case 3:
                leg4Servo.write(legAngles[i]);
                break;
        }
    }
}
