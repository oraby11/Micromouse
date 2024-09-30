
#include <Wire.h>
#include<I2Cdev.h>
#include <MPU6050.h>
MPU6050 gyro;

const int16_t speed= 200;
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

// تعريف Pins للموتورين
const int IN1 = 10;  // Pin للتحكم في اتجاه الموتور اليمين (أمام)
const int IN2 = 12;  // Pin للتحكم في اتجاه الموتور اليمين (خلف)
const int motorRightSpeed = 9;  // Pin للتحكم في سرعة الموتور اليمين (PWM)

const int IN3 = 25;  // Pin للتحكم في اتجاه الموتور الشمال (أمام)
const int IN4 = 28;  // Pin للتحكم في اتجاه الموتور الشمال (خلف)
const int motorLeftSpeed = 10;  // Pin للتحكم في سرعة الموتور الشمال (PWM)

//Variable to save readings
bool  SensorForward,SensorRight,SensorLeft;

//Variables of Encoder
float wheelDiameter = 0.06; // diametar wheel
int pulsenumber = 360; // number of pulse
float cellSize = 0.18; // distance cell


int  oldLeftPosition = 0;
int  oldRightPosition = 0;
float distance = 0;

   // متغيرات لتخزين القيم القادمة من الجيروسكوب
  int16_t gyroX, gyroY, gyroZ;
const int gyroAddress = 0x68; // عنوان الجيروسكوب
int16_t gyroOffsetZ = 0;

typedef enum {
   FRONT,
   RIGHT,
   LEFT,
}type_direction;

void setup() {
  
  Serial.begin(9600);
  setupGyroscope(); // إعداد الجيروسكوب
  setupMotors();    // إعداد الموتورات
    
  // معايرة الجيروسكوب في بداية التشغيل
  calibrateGyroscope();
}

void loop() {
  
  ReadingSensor();
  ReadEncoder();
  

  // قراءة القيم من الجيروسكوب
  readGyroscope(gyroX, gyroY, gyroZ);
  controlMotors( gyroZ, speed);
  

  // طباعة القيم في السيريال للمراقبة
  /*Serial.print("X: "); Serial.print(gyroX);
  Serial.print(" Y: "); Serial.print(gyroY);
  Serial.print(" Z: "); Serial.println(gyroZ);*/
  
  // التحكم في الموتور باستخدام PID
  controlMotorsWithPID(gyroZ);

  
  delay(100); // تأخير بسيط قبل التكرار
}

void ReadingSensor()
{
   //Read IR Reading 
SensorForward=digitalRead(IR_FORWARD);
SensorRight=digitalRead(IR_RIGHT);
SensorLeft=digitalRead(IR_LEFT);

  SensorForward ? Serial.print("Wall forward  ") : Serial.print("No wall forward  ");
  SensorRight ? Serial.print("Wall right  ") : Serial.print("No wall right  ");
  SensorLeft ? Serial.print("Wall left  ") : Serial.print("No wall left  ");
  delay(100);
}


// إعداد الجيروسكوب
void setupGyroscope() {
  Wire.begin(); // بدء الاتصال I2C
  Wire.beginTransmission(gyroAddress); 
  Wire.write(0x6B); // أوامر البدء
  Wire.write(0);    
  Wire.endTransmission(true);
}

// دالة لقراءة قيم الجيروسكوب
void readGyroscope(int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ) {
  Wire.beginTransmission(gyroAddress);
  Wire.write(0x3B); // طلب بيانات الجيروسكوب
  Wire.endTransmission(false);
  Wire.requestFrom(gyroAddress, 6, true);

  // قراءة القيم في المحاور الثلاثة
  gyroX = Wire.read() << 8 | Wire.read(); 
  gyroY = Wire.read() << 8 | Wire.read(); 
  gyroZ = Wire.read() << 8 | Wire.read(); 
}



void calibrateGyroscope() {
  int readings = 200;
  long sumZ = 0;
  
  for (int i = 0; i < readings; i++) {
    int16_t gyroX, gyroY, gyroZ;
    readGyroscope(gyroX, gyroY, gyroZ);
    sumZ += gyroZ;
    delay(3); // ندي وقت للجيروسكوب يثبت
  }
  
  // نحسب المتوسط كقيمة مرجعية للميلان في الوضع الثابت
  gyroOffsetZ = sumZ / readings;
}
/*******************************************************************/
/******************************************************************/

// إعداد المخارج الخاصة بالموتورات
void setupMotors() {
  // إعداد Pins الاتجاه للموتور اليمين
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(motorRightSpeed, OUTPUT);
  
  // إعداد Pins الاتجاه للموتور الشمال
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(motorLeftSpeed, OUTPUT);
}

// التحكم في الموتورات بناءً على قيمة الجيروسكوب
void controlMotors(int16_t gyroZ, int speed) {
  switch (type_direction)
  {
    case FRONT:
    gyroZ-=0;
  if (gyroZ< -1) { // ميلان لليمين
    turnLeft(speed);
  } else if (gyroZ > 1) { // ميلان لليسار
    turnRight(speed);
  } else { // لا يوجد ميلان كبير
    
    moveForward(speed);
  }break;

  case RIGHT:
        gyroZ-=90;
  if (gyroZ< -1) { // ميلان لليمين
    turnLeft(speed);
  } else if (gyroZ > 1) { // ميلان لليسار
    turnRight(speed);
  } else { // لا يوجد ميلان كبير
    
    moveForward(speed);
  }break;

   case LEFT:
        gyroZ+=90;
  if (gyroZ< -1) { // ميلان لليمين
    turnLeft(speed);
  } else if (gyroZ > 1) { // ميلان لليسار
    turnRight(speed);
  } else { // لا يوجد ميلان كبير
    
    moveForward(speed);
  }break;
  
}
}
// تحريك الموتور للأمام
void moveForward(int speed) {
  // تحريك الموتور اليمين للأمام
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(motorRightSpeed, speed); // التحكم في السرعة
  
  // تحريك الموتور الشمال للأمام
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(motorLeftSpeed, speed);  // التحكم في السرعة
}

// لف يمين
void turnRight(int speed) {
  // الموتور اليمين يتوقف
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(motorRightSpeed, 0);  // إيقاف الموتور اليمين
  
  // الموتور الشمال يتحرك للأمام
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(motorLeftSpeed, speed);  // سرعة الموتور الشمال
}

// لف شمال
void turnLeft(int speed) {
 
  // الموتور اليمين يتحرك للأمام
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(motorRightSpeed, speed);  // سرعة الموتور اليمين

   // الموتور الشمال يتوقف
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(motorLeftSpeed, 0);  // إيقاف الموتور الشمال
  
}

/**********************************************************************/
/*********************************************************************/

float lastError = 0;
float integral = 0;

// دالة PID للتحكم في الموتور بناءً على الجيروسكوب
void controlMotorsWithPID(int16_t gyroZ) {
  float Kp = 0.1; // عامل التناسب
  float Ki = 0.01; // عامل التكامل
  float Kd = 0.01; // عامل المشتقة

  // حساب الخطأ بعد تصحيح الإزاحة
  float error = gyroZ - gyroOffsetZ;

  // حساب التكامل (للتراكمات بمرور الوقت)
  integral += error;

  // حساب المشتقة (للتغيرات المفاجئة)
  float derivative = error - lastError;
  
  // حساب الناتج النهائي
  float output = Kp * error + Ki * integral + Kd * derivative;
  
  // تعديل السرعات بناءً على الناتج
  int motorRightSpeed = 255 - output;
  int motorLeftSpeed = 255 + output;

  // التأكد إن السرعات ما تعديش الحد الأقصى أو الأدنى
  motorRightSpeed = constrain(motorRightSpeed, 0, 255);
  motorLeftSpeed = constrain(motorLeftSpeed, 0, 255);

  analogWrite(motorRightSpeed,speed);
  analogWrite(motorLeftSpeed,speed);

  lastError = error; // تخزين الخطأ الأخير
}

/*********************************************/
  // reading of Encoder
  void ReadEncoder()
  {
    
   distance = 0;

  // check distance cell
  while(distance != cellSize || distance>cellSize) {
    
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


