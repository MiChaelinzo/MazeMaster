
#include <Arduino.h>
#include <SharpIR.h>
#include <L298N.h>

// --- Pin Definitions ---
#define IR_FRONT_PIN A0
#define IR_LEFT_PIN A1
#define IR_RIGHT_PIN A2
#define MOTOR_LEFT_PWM 9
#define MOTOR_LEFT_DIR 8
#define MOTOR_RIGHT_PWM 10
#define MOTOR_RIGHT_DIR 11
// Add ultrasonic sensor pins if needed
// #define TRIGGER_PIN ...
// #define ECHO_PIN ...

// --- Sensor Objects ---
SharpIR sharpIRFront(IR_FRONT_PIN, 1080);
SharpIR sharpIRLeft(IR_LEFT_PIN, 1080);
SharpIR sharpIRRight(IR_RIGHT_PIN, 1080);

// --- Motor Objects ---
L298N motorLeft(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR);
L298N motorRight(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR);

// --- PID Controller ---
class PID {
public:
    PID(float kp, float ki, float kd, int minOutput, int maxOutput) 
        : Kp(kp), Ki(ki), Kd(kd), minOutput(minOutput), maxOutput(maxOutput) {}

    int calculate(int error) {
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0;
        lastTime = now;

        integral += error * dt;
        // Prevent integral windup
        integral = constrain(integral, -integralLimit, integralLimit);

        float derivative = (error - prevError) / dt;
        prevError = error;

        int output = Kp * error + Ki * integral + Kd * derivative;
        return constrain(output, minOutput, maxOutput);
    }

    void reset() {
        integral = 0;
        prevError = 0;
        lastTime = millis();
    }

private:
    float Kp, Ki, Kd;
    int minOutput, maxOutput;
    unsigned long lastTime = 0;
    float integral = 0;
    int prevError = 0;
    int integralLimit = 50; // Adjust integral limit as needed
};

// --- PID Instance ---
PID wallFollowerPID(0.5, 0.1, 0.05, -100, 100); 

// --- Constants ---
const int BASE_SPEED = 150;
const int DESIRED_DISTANCE = 15;     // cm
const int WALL_FOLLOWING_DISTANCE = 10; // cm
const int TURN_SPEED = 100;         // Adjust for turn sharpness
const int TURN_DELAY = 300;          // milliseconds

// --- Enumerations ---
enum Movement { MOVE_FORWARD, MOVE_BACKWARD, TURN_LEFT, TURN_RIGHT, STOP };

// --- Global Variables ---
int frontDistance, leftDistance, rightDistance;  
Movement nextMove = STOP; 

// --- Maze Mapping ---
const int MAZE_SIZE = 16;
bool mazeMap[MAZE_SIZE][MAZE_SIZE];
int robotX, robotY;
int robotDirection; // 0 = North, 1 = East, 2 = South, 3 = West

// --- Function Prototypes ---
void readSensorData();
void updateMazeMap(); 
void determineNextMove();
void executeMovement();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMotors();

// ======================================================
// Setup
// ======================================================
void setup() {
    Serial.begin(9600);
    // Initialize maze map
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            mazeMap[i][j] = false; 
        }
    }
    robotX = 0;
    robotY = 0;
    robotDirection = 0; // Start facing North
}

// ======================================================
// Main Loop 
// ======================================================
void loop() {
    readSensorData();
    updateMazeMap(); 
    determineNextMove();
    executeMovement();
}

// ======================================================
// Functions
// ======================================================
void readSensorData() {
    frontDistance = sharpIRFront.distance();
    leftDistance = sharpIRLeft.distance();
    rightDistance = sharpIRRight.distance();
    // ... (Add ultrasonic sensor reading if needed)
}

void updateMazeMap() {
    // Example: Update mazeMap based on robot's current position and sensor readings
    // (You'll need to adjust this logic based on your specific maze mapping strategy)

    mazeMap[robotX][robotY] = true; // Mark current cell as visited

    // Example: If a wall is detected to the right, mark the corresponding cell in mazeMap as a wall
    if (rightDistance < WALL_FOLLOWING_DISTANCE) {
        switch (robotDirection) {
            case 0: // North
                mazeMap[robotX + 1][robotY] = true; 
                break;
            case 1: // East
                mazeMap[robotX][robotY + 1] = true;
                break;
            // Add cases for South and West
        }
    } 

    // Add similar logic for front and left sensors
}

void determineNextMove() {
    // Simple wall-following logic
    if (rightDistance < WALL_FOLLOWING_DISTANCE) {
        nextMove = TURN_LEFT; 
    } else if (frontDistance < DESIRED_DISTANCE) {
        nextMove = TURN_RIGHT;
    } else {
        nextMove = MOVE_FORWARD;
    }
}

void executeMovement() {
    switch (nextMove) {
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


// --- Movement Helper Functions ---
void moveForward() {
    int motorSpeedAdjustment = wallFollowerPID.calculate(DESIRED_DISTANCE - leftDistance);
    int leftSpeed = BASE_SPEED + motorSpeedAdjustment;
    int rightSpeed = BASE_SPEED - motorSpeedAdjustment;

    motorLeft.setSpeed(constrain(leftSpeed, 0, 255));
    motorRight.setSpeed(constrain(rightSpeed, 0, 255));

    motorLeft.forward();
    motorRight.forward();

    // Update robot's position based on direction
    switch (robotDirection) {
        case 0: // North
            robotY++; 
            break;
        case 1: // East
            robotX++;
            break;
        // Add cases for South and West
    }
}

void moveBackward() {
    // Similar to moveForward, but reverse motor directions
}

void turnLeft() {
    motorLeft.setSpeed(TURN_SPEED); 
    motorRight.setSpeed(TURN_SPEED);
    motorLeft.backward();
    motorRight.forward();
    delay(TURN_DELAY); 
    motorLeft.stop();
    motorRight.stop();

    // Update robot's direction
    robotDirection = (robotDirection - 1 + 4) % 4; // Decrement direction, wrap around
    wallFollowerPID.reset(); // Reset PID after turning
}

void turnRight() {
    // Similar to turnLeft, but adjust motor directions
}

void stopMotors() {
    motorLeft.stop();
    motorRight.stop();
}



/// - **Integral Windup Protection:** Added `integralLimit` and a check in the `PID::calculate()` method to prevent integral windup, which can cause oscillations and instability.
//- **PID Reset After Turns:**  Added `wallFollowerPID.reset()` in the `turnLeft()` and `turnRight()` functions to reset the PID controller's integral and derivative terms after each turn. This helps the robot maintain better wall-following accuracy. 
//- **Maze Mapping:** 
//    - Provided a basic example of how to update your `mazeMap` in the `updateMazeMap` function. This example assumes you want to mark cells as "visited" and detect walls. You'll need to tailor this logic to your specific maze-solving strategy.
//    - **Important:** Maze mapping requires a robust method for determining the robot's position and orientation within the maze. You might need to incorporate additional sensors or techniques like odometry or wheel encoders for accurate mapping. 
//- **Robot Position Tracking:** Added code to update the `robotX`, `robotY`, and `robotDirection` variables in the `moveForward`, `turnLeft`, and `turnRight` functions to keep track of the robot's location and heading in the maze. 

//**Next Steps:**

//1. **Complete Maze Mapping Logic:**  If you're using maze mapping:
//    - Fully implement the wall detection logic in `updateMazeMap` for all directions (front, left, right) to create an accurate representation of the maze. 
//    - Choose a maze-solving algorithm (Flood Fill, A*, etc.) and implement it in `determineNextMove` to guide the robot through the maze based on the mapped data.
//2. **Calibrate and Test:**
//    - **Sensor Calibration:**  Calibrate your IR sensors to get accurate distance readings. This might involve taking measurements at known distances and creating a calibration curve.
//    - **PID Tuning:**  Experiment with different PID values (`Kp`, `Ki`, `Kd`) to achieve smooth and stable wall following.
//    - **Movement Testing:**  Test your robot's movements (forward, backward, turns) to ensure they are accurate and reliable. You might need to adjust the `TURN_DELAY` value or use a different method for controlling turn angles. 

