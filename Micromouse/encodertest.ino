

// Motor pin definitions
const int leftMotorPin1 = 26; 
const int leftMotorPin2 = 14;
const int rightMotorPin1 = 19;
const int rightMotorPin2 = 32;

// IR sensor pins
const int leftIRPin = 4;   // Left encoder على D2 (GPIO 4)
const int rightIRPin = 16; // Right encoder على GPIO 16

volatile int leftPulseCount = 0;
volatile int rightPulseCount = 0;
const float targetDistance = 18.5;    
unsigned int pulse_per_turn = 22; // Depends on the number of slots on the slotted disc
float wheel_circumference = 25.390; // 2*pi*radius 
//2*pi*4.43
void IRAM_ATTR countLeftPulses() {
  leftPulseCount++;  
}

void IRAM_ATTR countRightPulses() {
  rightPulseCount++;  
}

void setup() {
  leftPulseCount = 0;
  rightPulseCount=0;
  Serial.begin(115200);
    // Initialize motor pins
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  pinMode(leftIRPin, INPUT);
  pinMode(rightIRPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(leftIRPin), countLeftPulses, RISING);
  attachInterrupt(digitalPinToInterrupt(rightIRPin), countRightPulses, RISING);  
   
}

 
void loop() 
{
  moveForward();
 float leftDistance=calculateDistanceLeft();
 float rightDistance=calculateDistanceRight();
 float average=(leftDistance+rightDistance) /2;
 if (average>=targetDistance)
 {
  stopMotors();
 }


}
  float calculateDistanceLeft() 
   {
    
  // Calculate distance traveled for the left motor in cm
  return (leftPulseCount * wheel_circumference) / pulse_per_turn;
   }

float calculateDistanceRight()
 {
  // Calculate distance traveled for the right motor in cm
  return (rightPulseCount * wheel_circumference) / pulse_per_turn;

}
void moveForward() {
  //leftDistance=0;
  //rightDistance=0;   
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
}
void stopMotors() {
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);

  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2,LOW);
  delay(500);
}

