#include "cells.h"
#include "fifo.h"

#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1_bc.h>
#include <Arduino.h>

// Motor pin definitions
const int leftMotorPin1 = 25; 
const int leftMotorPin2 = 7;
const int rightMotorPin1 = 10;
const int rightMotorPin2 = 12;

// HC-89 sensor pin definitions
const int leftHCSensorPin = 31;  // مستشعر HC-89 المتصل بمحور المحرك الأيسر
const int rightHCSensorPin = 29;  // مستشعر HC-89 المتصل بمحور المحرك الأيمن

volatile long leftPulseCount = 0;  // عداد نبضات المحرك الأيسر
volatile long rightPulseCount = 0; // عداد نبضات المحرك الأيمن

const int frontIRPin = 8;  // Front IR sensor
const int rightIRPin = 17;  // Right IR sensor
const int leftIRPin = 5;   // Left IR sensor

// Gyroscope setup
MPU6050 gyro;
bool firstReading = true;  // flag  لتحديد إذا كانت أول قراءة أو لا
float gyroOffsetZ = 0;
// PWM settings
int speed = 100;  // Speed should be between 0 and 255

// PID settings
double kp = 2.0, ki = 5.0, kd = 1.0; // PID coefficients
double setpoint = 0.0; // Desired value
double input = 0.0;    // Current value
double output = 0.0;   // Output to motors

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// Wheel settings
const float pi = 3.14159;
const float wheelDiameter = 443.938;  // Wheel diameter in millimeters
const int encoderSlots = 22;  // Number of slots in the wheel (تعتمد على تصميم القرص)
const float wheelCircumference = pi * wheelDiameter;  // Wheel circumference in millimeters
const float distancePerPulse = wheelCircumference / encoderSlots;  // Distance per pulse in millimeters
const int targetDistance = 185;  // Target distance 18.5 cm (185 mm)
const float tolerance = 1.0;  // Angle tolerance in degrees

FIFO_buffer_t queue;
robot_position_t head_pos = head_to_right;

element_type queue_buffer[fifo_size];
cell_t g_maze[MAZE_SIZE][MAZE_SIZE];
cell_t *stp = &g_maze[0][0];

// Interrupt service routines (ISR) for counting pulses
void IRAM_ATTR countLeftPulses() {
  leftPulseCount++;
}

void IRAM_ATTR countRightPulses() {
  rightPulseCount++;
}

void moveForward() {
  leftPulseCount = 0;
  rightPulseCount = 0;

  setpoint = targetDistance; // Set the target distance
  myPID.SetMode(AUTOMATIC);

  while (leftPulseCount * distancePerPulse < targetDistance && rightPulseCount * distancePerPulse < targetDistance) {
    // Calculate error and use PID to adjust speed
    input = leftPulseCount * distancePerPulse; // Adjust input as needed
    myPID.Compute();

    // Adjust motor speeds based on PID output
    analogWrite(leftMotorPin1, constrain(speed + output, 0, 255));  // Left motor
    analogWrite(rightMotorPin1, constrain(speed - output, 0, 255));  // Right motor

    delay(10); // Short delay to prevent excessive PID calculations
  }
  stopMotors();
}

void calibrateGyroscope() {
  int readings = 200;
  long sumZ = 0;

  for (int i = 0; i < readings; i++) {
    int16_t gyroX, gyroY, gyroZ;
    gyro.getRotation(&gyroX, &gyroY, &gyroZ);
    sumZ += gyroZ;
    delay(3); // ندي وقت للجيروسكوب يثبت
  }
  // نحسب المتوسط كقيمة مرجعية للميلان في الوضع الثابت
  gyroOffsetZ = sumZ / readings;
}

float getGyroAngle() {
  int16_t gx, gy, gz;
  gyro.getRotation(&gx, &gy, &gz);  // نجيب القراءة الحالية

  if (firstReading) {
    // أول قراءة: نطرح منها الإزاحة (offsetZ)
    gz -= gyroOffsetZ;
    firstReading = false;  // نخلي الفلاغ يشير إن أول قراءة تمت
  }

  // نرجع القراءة (مع أو بدون طرح الإزاحة بناءً على أول قراءة)
  return gz;  // زاوية الدوران حول محور Z
}

void adjustMotors(float error, bool isLeft) {
  int baseSpeed = speed;
  int adjustment = map(abs(error), 0, 90, 0, 50);

  if (isLeft) {
    analogWrite(leftMotorPin1, constrain(baseSpeed - adjustment, 0, 255));  // Adjust left motor speed
    analogWrite(rightMotorPin1, constrain(baseSpeed, 0, 255));               // Keep right motor speed
  } else {
    analogWrite(leftMotorPin1, constrain(baseSpeed, 0, 255));               // Keep left motor speed
    analogWrite(rightMotorPin1, constrain(baseSpeed - adjustment, 0, 255));  // Adjust right motor speed
  }
}

void moveMotorsForward() {
  // Set direction: LOW for forward
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin2, LOW);

  // Control speed using analogWrite
  analogWrite(leftMotorPin1, speed);
  analogWrite(rightMotorPin1, speed);
}

void stopMotors() {
  for (int i = speed; i > 0; i--) {
    analogWrite(leftMotorPin1, i);  // Stop left motor
    analogWrite(rightMotorPin1, i);  // Stop right motor
  }
}

void turnRight() {
  float initialAngle = getGyroAngle();
  setpoint = initialAngle + 90; // Set the target angle

  myPID.SetMode(AUTOMATIC);

  while (abs(getGyroAngle() - initialAngle) < 90) {
    input = getGyroAngle();
    myPID.Compute();

    // Adjust motor directions and speeds based on PID output
    digitalWrite(leftMotorPin2, LOW);
    digitalWrite(rightMotorPin2, HIGH);
    analogWrite(leftMotorPin1, constrain(speed + output, 0, 255));
    analogWrite(rightMotorPin1, constrain(speed - output, 0, 255));

    delay(10); // Short delay to prevent excessive PID calculations
  }
  stopMotors();
  delay(10);
}

void turnLeft() {
  float initialAngle = getGyroAngle();
  setpoint = initialAngle - 90; // Set the target angle

  myPID.SetMode(AUTOMATIC);

  while (abs(getGyroAngle() - initialAngle) < 90) {
    input = getGyroAngle();
    myPID.Compute();

    // Adjust motor directions and speeds based on PID output
    digitalWrite(leftMotorPin2, HIGH);
    digitalWrite(rightMotorPin2, LOW);
    analogWrite(leftMotorPin1, constrain(speed - output, 0, 255));
    analogWrite(rightMotorPin1, constrain(speed + output, 0, 255));

    delay(10); // Short delay to prevent excessive PID calculations
  }
  stopMotors();
  delay(10);
}

bool readFrontIR() {
  return digitalRead(frontIRPin); // Read front IR sensor value
}

bool readRightIR() {
  return digitalRead(rightIRPin); // Read right IR sensor value
}

bool readLeftIR() {
  return digitalRead(leftIRPin); // Read left IR sensor value
}

void setup() {
  // Initialize motor pins
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  // Initialize gyroscope
  Wire.begin(33, 36);  // SDA and SCL on the specified pins
  gyro.initialize();

  calibrateGyroscope();
  // Initialize PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255); // Set output limits

  // Initialize Serial
  Serial.begin(19200);

  // Initialize FIFO and LIFO
  FIFO_init(&queue, queue_buffer, fifo_size);
  init_all_cells(g_maze, (uint16_t)MAZE_SIZE);

  // Setup interrupts for pulse counting
  attachInterrupt(digitalPinToInterrupt(leftHCSensorPin), countLeftPulses, RISING);
  attachInterrupt(digitalPinToInterrupt(rightHCSensorPin), countRightPulses, RISING);
}

void loop() {
  if (stp->cell_val == 0) {
    head_pos = head_to_right;
    stp = &g_maze[0][0];
    delay(10000);
  }
  goto_smallest_val(g_maze, stp);
}
