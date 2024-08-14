#include <Arduino.h>
#include <L298N.h> 
#include <SharpIR.h> 
#include <HCSR04.h> 
// #include <LidarLibrary.h> // If using LiDAR, uncomment and include the library

// --- Pin Definitions ---
#define IR_FRONT_PIN A0
#define IR_LEFT_PIN A1
#define IR_RIGHT_PIN A2
#define TRIGGER_PIN 7
#define ECHO_PIN 8
#define MOTOR_LEFT_PWM 9
#define MOTOR_LEFT_DIR 8
#define MOTOR_RIGHT_PWM 10
#define MOTOR_RIGHT_DIR 11
#define BATTERY_PIN A3     // Example analog pin for battery voltage
#define LED_BUILTIN 13
// ... (Define pins for LiDAR if used)

// --- Sensor Objects ---
SharpIR sharpIRFront(IR_FRONT_PIN, 1080);
SharpIR sharpIRLeft(IR_LEFT_PIN, 1080);
SharpIR sharpIRRight(IR_RIGHT_PIN, 1080);
UltraSonicDistanceSensor distanceSensor(TRIGGER_PIN, ECHO_PIN);
// Lidar lidar(...); // Uncomment and create LiDAR object if used

// --- Motor Objects ---
L298N motorLeft(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR);
L298N motorRight(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR);

// --- Constants ---
const int MAZE_SIZE = 16;
const int CELL_SIZE = 10; // Assume each cell in the maze is 10 cm (adjust as needed)
const int WALL_THRESHOLD = 10; // Distance (cm) at which a wall is detected

// --- Enumerations ---
enum CellType { UNKNOWN, OPEN, WALL };
enum Direction { North, East, South, West };
enum Movement { MOVE_FORWARD, MOVE_BACKWARD, TURN_LEFT, TURN_RIGHT, STOP };

// --- Robot State ---
struct Robot {
  int x;
  int y;
  Direction direction;
} robot;

// --- Maze Representation ---
CellType mazeMap[MAZE_SIZE][MAZE_SIZE];

// --- Movement Constants ---
const int BASE_SPEED = 150;
const int TURN_SPEED = 100;
const int TURN_DELAY = 300; // Adjust for desired turn angle
const float VOLTAGE_DIVIDER_RATIO = 2.0; // Adjust based on your voltage divider circuit
const float BATTERY_LOW_THRESHOLD = 6.0; // Adjust for your battery type

// --- Global Variables ---
int frontDistance, leftDistance, rightDistance;
long ultrasonicDistance; 
Movement nextMove = STOP;

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
bool checkForErrors();
void handleError();

// ======================================================
// Setup
// ======================================================
void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize maze map
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      mazeMap[i][j] = UNKNOWN; 
    }
  }

  // Initialize robot state
  robot.x = CELL_SIZE / 2;    // Start in the middle of the first cell
  robot.y = CELL_SIZE / 2; 
  robot.direction = North;
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
  frontDistance = sharpIRFront.distance();
  leftDistance = sharpIRLeft.distance();
  rightDistance = sharpIRRight.distance();

  ultrasonicDistance = distanceSensor.measureDistanceCm(); 

  // ... (Read LiDAR data if used)
}

void updateMazeMap() {
  // --- Simplified maze mapping using IR sensors ---
  int cellX = robot.x / CELL_SIZE;
  int cellY = robot.y / CELL_SIZE;

  // --- Boundary checks to prevent out-of-bounds array access ---
  cellX = constrain(cellX, 0, MAZE_SIZE - 1);
  cellY = constrain(cellY, 0, MAZE_SIZE - 1);

  switch (robot.direction) {
    case North:
      if (frontDistance < WALL_THRESHOLD) mazeMap[cellX][cellY + 1] = WALL;
      else if (frontDistance < 3 * CELL_SIZE) mazeMap[cellX][cellY + 1] = OPEN; 
      if (leftDistance < WALL_THRESHOLD) mazeMap[cellX - 1][cellY] = WALL;
      else if (leftDistance < 3 * CELL_SIZE) mazeMap[cellX - 1][cellY] = OPEN;
      if (rightDistance < WALL_THRESHOLD) mazeMap[cellX + 1][cellY] = WALL;
      else if (rightDistance < 3 * CELL_SIZE) mazeMap[cellX + 1][cellY] = OPEN;
      break;
    case East: 
      // ... (Similar logic for East, South, and West)
      break;
    // ... (Add cases for South and West)
  }
}


void determineNextMove() {
  // --- Example: Left-hand wall following --- 
  if (leftDistance > 15) {
    nextMove = TURN_LEFT;
  } else if (frontDistance < 10) {
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
  motorLeft.setSpeed(BASE_SPEED);
  motorRight.setSpeed(BASE_SPEED);
  motorLeft.forward();
  motorRight.forward();

  // --- Update robot position based on direction ---
  switch (robot.direction) {
    case North: robot.y += CELL_SIZE; break;
    case East:  robot.x += CELL_SIZE; break;
    case South: robot.y -= CELL_SIZE; break;
    case West:  robot.x -= CELL_SIZE; break;
  } 
}

void moveBackward() {
  // ... (Implementation for moving backward)
}

void turnLeft() {
  motorLeft.setSpeed(TURN_SPEED);
  motorRight.setSpeed(TURN_SPEED);
  motorLeft.backward();
  motorRight.forward();
  delay(TURN_DELAY);
  stopMotors();

  // --- Update robot direction ---
  robot.direction = (Direction)((robot.direction + 3) % 4); // Rotate counter-clockwise
}

void turnRight() {
  motorLeft.setSpeed(TURN_SPEED);
  motorRight.setSpeed(TURN_SPEED);
  motorLeft.forward();
  motorRight.backward();
  delay(TURN_DELAY);
  stopMotors();

  // --- Update robot direction ---
  robot.direction = (Direction)((robot.direction + 1) % 4); // Rotate clockwise
}

void stopMotors() {
  motorLeft.stop();
  motorRight.stop();
}

bool checkForErrors() {
  if (sharpIRFront.distance() == 0 || 
      sharpIRLeft.distance() == 0 || 
      sharpIRRight.distance() == 0 || 
      distanceSensor.measureDistanceCm() == 0) {
    Serial.println("Error: Sensor failure");
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
