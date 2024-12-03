#include <NewPing.h>

// Define Ultrasonic Sensor Pins
#define TRIG_FRONT 2
#define ECHO_FRONT 3
#define TRIG_LEFT 4
#define ECHO_LEFT 5
#define TRIG_RIGHT 6
#define ECHO_RIGHT 7

// Define Motor Pins
#define IN1 8
#define IN2 9
#define ENA 10
#define IN3 12
#define IN4 13
#define ENB 11

// Motor Speed Control
#define MOTOR_SPEED 150

// Maze parameters
#define MAZE_SIZE 16  // 16x16 grid
#define MAX_VALUE 255

// Define goal position in the maze (center)
#define GOAL_X 7  // Adjust based on your maze
#define GOAL_Y 7  // Adjust based on your maze

// Ultrasonic Range
#define MAX_DISTANCE 200
NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);
NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, MAX_DISTANCE);

// Flood Fill Maze Array
byte maze[MAZE_SIZE][MAZE_SIZE];
byte posX = 0, posY = 0;  // Start position
byte direction = 0;       // 0=North, 1=East, 2=South, 3=West

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  Serial.begin(9600);
  floodFillInit();
}

void loop() {
  floodFill(posX, posY);
  moveToGoal();
  returnToStart();
  moveToGoal();
}

// Initialize Flood Fill maze with MAX_VALUE
void floodFillInit() {
  for (byte i = 0; i < MAZE_SIZE; i++) {
    for (byte j = 0; j < MAZE_SIZE; j++) {
      maze[i][j] = MAX_VALUE;
    }
  }
  maze[GOAL_X][GOAL_Y] = 0;
}

// Function to get distance from ultrasonic sensor
int getDistance(NewPing sensor) {
  delay(50);  // Small delay between readings
  return sensor.ping_cm();
}

// Move robot forward
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, MOTOR_SPEED);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, MOTOR_SPEED);
}

// Stop the robot
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

// Turn robot left
void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, MOTOR_SPEED);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, MOTOR_SPEED);
  delay(450); // Adjust to achieve 90-degree turn
}

// Turn robot right
void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, MOTOR_SPEED);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, MOTOR_SPEED);
  delay(450); // Adjust to achieve 90-degree turn
}

// Update the robot's direction
void updateDirection(byte newDirection) {
  if (direction != newDirection) {
    if ((newDirection + 3) % 4 == direction) {
      turnLeft();
      turnLeft();
    } else if ((newDirection + 1) % 4 == direction) {
      turnRight();
    } else {
      turnLeft();
    }
  }
  direction = newDirection;
}

// Flood fill algorithm for solving the maze
void floodFill(byte x, byte y) {
  while (true) {
    int frontDistance = getDistance(sonarFront);
    int leftDistance = getDistance(sonarLeft);
    int rightDistance = getDistance(sonarRight);

    // Detect open paths and update movement
    if (frontDistance > 15) {  // Adjust threshold for your maze
      moveForward();
      posX += (direction == 1) - (direction == 3);
      posY += (direction == 0) - (direction == 2);
    } else {
      stopMotors();  // Stop before making a turn
      if (leftDistance > 15) {
        updateDirection((direction + 3) % 4);
        moveForward();
      } else if (rightDistance > 15) {
        updateDirection((direction + 1) % 4);
        moveForward();
      } else {
        updateDirection((direction + 2) % 4); // Turn around and move forward
        // The following line is commented out to prevent backward movement
        // moveForward();
      }
    }
    
    // Update maze and apply flood fill logic
    byte minValue = min(maze[posX+1][posY], min(maze[posX-1][posY], min(maze[posX][posY+1], maze[posX][posY-1])));
    maze[posX][posY] = minValue + 1;
    
    // Stop if goal is reached
    if (posX == GOAL_X && posY == GOAL_Y) break;
  }
}

// Move to the goal using the shortest path
void moveToGoal() {
  while (posX != GOAL_X || posY != GOAL_Y) {
    byte minValue = maze[posX][posY];
    byte nextDirection = direction;

    if (posY > 0 && maze[posX][posY - 1] < minValue && direction != 2) { // Prevent moving backward
      minValue = maze[posX][posY - 1];
      nextDirection = 0;  // North
    }
    if (posX < MAZE_SIZE - 1 && maze[posX + 1][posY] < minValue && direction != 3) { // Prevent moving backward
      minValue = maze[posX + 1][posY];
      nextDirection = 1;  // East
    }
    if (posY < MAZE_SIZE - 1 && maze[posX][posY + 1] < minValue && direction != 0) { // Prevent moving backward
      minValue = maze[posX][posY + 1];
      nextDirection = 2;  // South
    }
    if (posX > 0 && maze[posX - 1][posY] < minValue && direction != 1) { // Prevent moving backward
      minValue = maze[posX - 1][posY];
      nextDirection = 3;  // West
    }

    updateDirection(nextDirection);
    moveForward();
  }
}

// Return to start position using the shortest path
void returnToStart() {
  // Similar logic as moveToGoal() but reversed
  while (posX != 0 || posY != 0) {
    byte minValue = maze[posX][posY];
    byte nextDirection = direction;

    if (posY > 0 && maze[posX][posY - 1] < minValue && direction != 2) { // Prevent moving backward
      minValue = maze[posX][posY - 1];
      nextDirection = 0;  // North
    }
    if (posX < MAZE_SIZE - 1 && maze[posX + 1][posY] < minValue && direction != 3) { // Prevent moving backward
      minValue = maze[posX + 1][posY];
      nextDirection = 1;  // East
    }
    if (posY < MAZE_SIZE - 1 && maze[posX][posY + 1] < minValue && direction != 0) { // Prevent moving backward
      minValue = maze[posX][posY + 1];
      nextDirection = 2;  // South
    }
    if (posX > 0 && maze[posX - 1][posY] < minValue && direction != 1) { // Prevent moving backward
      minValue = maze[posX - 1][posY];
      nextDirection = 3;  // West
    }

    updateDirection(nextDirection);
    moveForward();
  }
}
