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

const float distancePerPulse = 0.6342857142857143;
const float targetDistance = 19.2; 
int targetPulses; 
//4.444 dim
// m 13.93794285714286

void IRAM_ATTR countLeftPulses() {
  leftPulseCount++;
}

void IRAM_ATTR countRightPulses() {
  rightPulseCount++;
}

void setup() {
  // Initialize motor pins
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  // Initialize IR sensor pins
  pinMode(leftIRPin, INPUT);
  pinMode(rightIRPin, INPUT);

  // Attach interrupts for counting pulses
  attachInterrupt(digitalPinToInterrupt(leftIRPin), countLeftPulses, RISING);
  attachInterrupt(digitalPinToInterrupt(rightIRPin), countRightPulses, RISING);


  targetPulses = targetDistance / distancePerPulse;

  Serial.begin(115200);  
}

void loop() {


  // Start moving forward
  moveForward();
  stopMotors();
  // Wait for a bit before repeating the process
  delay(2000);
}

void moveForward() {
    // Reset pulse counts
  leftPulseCount = 0;
  rightPulseCount=0;
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  // Keep moving until both wheels have reached the target number of pulses
  while (rightPulseCount*distancePerPulse < targetDistance) {
    // Print the pulse counts and calculated distance for debugging
    Serial.print("Left pulse: ");
    Serial.print(leftPulseCount);
     Serial.print("right pulse: ");
    Serial.print(rightPulseCount);
  }

  // Stop the motors when distance is reached
  stopMotors();
   
}

void stopMotors() {
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);

  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2,LOW);
}
