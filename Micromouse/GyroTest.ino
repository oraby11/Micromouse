//#include "cells.h"
 //#include "fifo.h"

#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1_bc.h>
#include <Arduino.h>

// Motor pin definitions
const int leftMotorPin1 = 26; 
const int leftMotorPin2 = 14;
const int rightMotorPin1 = 19;
const int rightMotorPin2 = 32;

// HC-89 sensor pin definitions
const int leftHCSensorPin = 4;  // مستشعر HC-89 المتصل بمحور المحرك الأيسر
const int rightHCSensorPin = 16;  // مستشعر HC-89 المتصل بمحور المحرك الأيمن

volatile long leftPulseCount = 0;  // عداد نبضات المحرك الأيسر
volatile long rightPulseCount = 0; // عداد نبضات المحرك الأيمن

const int frontIRPin = 8;  // Front IR sensor
const int rightIRPin = 17;  // Right IR sensor
const int leftIRPin = 5;   // Left IR sensor

// Gyroscope setup
MPU6050 gyro;
bool firstReading = true;  // flag  لتحديد إذا كانت أول قراءة أو لا
float gyroOffsetX = 0;
// PWM settings
int speed = 200;  // Speed should be between 0 and 255

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
  long sumX = 0;

  for (int i = 0; i < readings; i++) {
    int16_t gyroX, gyroY, gyroZ;
    gyro.getRotation(&gyroX, &gyroY, &gyroZ);
    sumX += gyroX;
    delay(3); // ندي وقت للجيروسكوب يثبت
  }
  // نحسب المتوسط كقيمة مرجعية للميلان في الوضع الثابت
  gyroOffsetX = sumX / readings;
}

float getGyroAngle() {
  int16_t gx, gy, gz;
  gyro.getRotation(&gx, &gy, &gz);  // نجيب القراءة الحالية

  if (firstReading) {
    // أول قراءة: نطرح منها الإزاحة (offsetZ)
    gx -= gyroOffsetX;
    firstReading = false;  // نخلي الفلاغ يشير إن أول قراءة تمت
  }

  // نرجع القراءة (مع أو بدون طرح الإزاحة بناءً على أول قراءة)
  return gx;  // زاوية الدوران حول محور Z
}
void turnGyro(float targetAngle, bool turnLeft) {
  float currentAngle = 0;
  float previousAngle = 0;
  
  // Reset the first reading flag to ensure proper offset application
  firstReading = true;
  
  // تحديد الاتجاه بناءً على الفلاغ turnLeft
  if (turnLeft) {
    while (abs(currentAngle) < targetAngle) {
      currentAngle += getGyroAngle();  // احصل على الزاوية الحالية
      adjustMotors(targetAngle - abs(currentAngle), true);  // ضبط المحركات للالتفاف لليسار
      delay(10);  // تأخير صغير لضمان قراءة الجيروسكوب وتحديث الزاوية
    }
  } else {
    while (abs(currentAngle) < targetAngle) {
      currentAngle += getGyroAngle();  // احصل على الزاوية الحالية
      adjustMotors(targetAngle - abs(currentAngle), false);  // ضبط المحركات للالتفاف لليمين
      delay(10);  // تأخير صغير لضمان قراءة الجيروسكوب وتحديث الزاوية
    }
  }
  
  // بعد الوصول إلى الزاوية المطلوبة نوقف المحركات
  stopMotors();
  delay(500);  // تأخير بسيط بعد الانتهاء من الالتفاف
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

  

  // Setup interrupts for pulse counting
  attachInterrupt(digitalPinToInterrupt(leftHCSensorPin), countLeftPulses, RISING);
  attachInterrupt(digitalPinToInterrupt(rightHCSensorPin), countRightPulses, RISING);
}
void loop() {
  // تحرك للأمام لمسافة 18.5 سم
  moveForward();
  delay(500);  // انتظر نصف ثانية بعد التحرك للأمام

  // استدر لليسار بزاوية 90 درجة
  turnGyro(90.0, true);  // التفاف لليسار بزاوية 90 درجة
  
  // تحرك للأمام مرة أخرى لمسافة 18.5 سم
  moveForward();
  delay(500);

  // استدر لليمين بزاوية 90 درجة
  turnGyro(90.0, false);  // التفاف لليمين بزاوية 90 درجة
  
  delay(1000);  // تأخير قبل العودة إلى الحركة مرة أخرى
}






