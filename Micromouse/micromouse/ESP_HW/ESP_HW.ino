
#include <Wire.h>
#include<I2Cdev.h>
#include <MPU6050.h>
MPU6050 gyro;

#include <Encoder.h>
#define SPEED  80
//Motor pins
#define IN1 10
#define IN2 12
#define IN3 25
#define IN4 28
//Control speed pins
#define ENA 4
#define ENB 3

//IR pins
#define IR_LEFT 5
#define IR_FORWARD 6
#define IR_RIGHT 7


//Encoder pins 
#define LeftEncoder 32
#define RightEncoder 31

//Gyroscope pins
#define SDA 33
#define SCL 36


//Variable to save readings
bool  SensorForward,SensorRight,SensorLeft;

//Gyroscope variables
float yaw = 0; // زاوية الدوران
long lastTime = 0; // الوقت عشان نحسب التغير في الزمن
float gyroZOffset = 0; // تصحيح الانجراف


//Variables of Encoder
float wheelDiameter = 0.06; // diametar wheel
int pulsenumber = 20; // number of pulse
float cellSize = 0.18; // distance cell
int  oldLeftPosition = 0;
int  oldRightPosition = 0;
float distance = 0;

float target=0;
float error=0;



/************************/
void setup() {
  Serial.begin(115200);
  
  // Motor
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // IR pins
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_FORWARD, INPUT);
  pinMode(IR_RIGHT, INPUT);
  

//Encoder pins 
 pinMode(LeftEncoder, INPUT);  
 pinMode(RightEncoder, INPUT); 


   Wire.begin();
   gyro.initialize();
  
  if (gyro.testConnection()) {
    Serial.println("MPU6050 متوصل تمام!");
  } else {
    Serial.println("في مشكلة في توصيل الجيروسكوب.");
  }

  // تصحيح الانجراف في البداية
  calibrateGyro();



}

void loop() {

 /**************************/ 
   //Read IR Reading 
SensorForward=digitalRead(IR_FORWARD);
SensorRight=digitalRead(IR_RIGHT);
SensorLeft=digitalRead(IR_LEFT);

  SensorForward ? Serial.print("Wall forward  ") : Serial.print("No wall forward  ");
  SensorRight ? Serial.print("Wall right  ") : Serial.print("No wall right  ");
  SensorLeft ? Serial.print("Wall left  ") : Serial.print("No wall left  ");
  delay(100);
/*****************************/


  updateYaw(); // تحديث زاوية الدوران باستخدام الجيروسكوب

  // هنا بنعمل كل حالة حسب المطلوب:

  // مثال: لو عايز يمشي في خط مستقيم
  //moveforward()
  //moveStraight();

  // مثال: لو عايز يدور يمين
 
  // turnRight();
   //moveforward()
  //moveStraight();

  // مثال: لو عايز يدور شمال
 
  // turnLeft();
   //moveforward();
  //moveStraight();

  // مثال: لو عايز يعمل دورة كاملة
  // turnRight();
  // turnRight();
  //moveforward();
  //moveStraight();

  delay(10); // مهلة لتجنب التحميل الزائد
}

// تحديث زاوية الدوران (Yaw)
void updateYaw() {
  int16_t gx, gy, gz;
  long currentTime = millis();
  long elapsedTime = currentTime - lastTime;

  gyro.getRotation(&gx, &gy, &gz);
  float rateZ = (gz - gyroZOffset) / 131.0; // قراءة الجيروسكوب (لكل ثانية)
  
  yaw += rateZ * (elapsedTime / 1000.0); // حساب زاوية الدوران لتحويل الزمن من ثانيه الى مللى 
  lastTime = currentTime;

 error=yaw - target ;
//absolute yaw  greater than 1 
  
}

// تصحيح انحراف الجيروسكوب
void calibrateGyro() {
  int16_t gx, gy, gz;
  long sum = 0;

  for (int i = 0; i < 200; i++) {
    gyro.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(3);
  }

  gyroZOffset = sum / 200.0;
}

//Function for gyroscope

void moveStraight(float error) 
{
  while (error<=1 && error>=-1)
  {
  if (error > 1)
  {
    turnRight();

  }
  else if (error < -1) 
  {
     turnLeft();
  } 
  /*else 
  {
    Serial.println("Moving straight...");
    analogWrite(ENA, SPEED); 
    analogWrite(ENB, SPEED); 
  }*/
  }
}

/***************************************/
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA,SPEED ); 

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB,SPEED );
}

void turnRight( ) {
  //on right motor
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
   analogWrite(ENA,0 ); 
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB,SPEED  );
}

void turnLeft( ) {
  // on left motor
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA,SPEED ); 
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB,0 );


}

void StopMotor()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
 
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}
/*********************************************/
  // reading of Encoder
  void ReadEncoder()
  {
   distance = 0;

  // check distance cell
  while(distance <= cellSize) {
    
      //Read IR

  //Code 
  //if robot move forward
 // moveforward();

  //if robot turn right 
 // turnRight();
 // moveforward();

  //if robot turn left 
 // turnLeft();
 // moveforward();
    int newLeftPosition = digitalRead(LeftEncoder);
    int newRightPosition = digitalRead(RightEncoder);


  // count number of pulse 
  int  changeLeft = newLeftPosition - oldLeftPosition;
  int  changeRight = newRightPosition - oldRightPosition;

  // update values
  oldLeftPosition = newLeftPosition;
  oldRightPosition = newRightPosition;

  // count distance of robot
  float pulseDistance = (wheelDiameter * 3.14159) / pulsenumber;
  float leftDistance = changeLeft * pulseDistance;
  float rightDistance = changeRight * pulseDistance;

  // average
  distance += (leftDistance + rightDistance) / 2;
 


  }
}




