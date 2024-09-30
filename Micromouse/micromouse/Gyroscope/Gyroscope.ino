#include <Wire.h>
#include <MPU6050.h>  // مكتبة MPU6050

MPU6050 mpu;

//Gyroscope pins
#define SDA 33
#define SCL 36

//IR pins
#define IR_LEFT 5
#define IR_FORWARD 6
#define IR_RIGHT 7


//Encoder pins 
#define LeftEncoder 32
#define RightEncoder 31

#define   motorRightSpeed  9
#define   motorLeftSpeed  10
#define  IN1   10
#define  IN2   12
#define  IN3   25
#define  IN4   28


float yaw = 0;   // الزاوية الحالية حول المحور Z
float previousYaw = 0;
unsigned long previousTime = 0;
int16_t gyroOffsetZ = 0;





void setup() {
  Serial.begin(9600);
  
  // إعداد الموتورات
  setupMotors();

  // بدء الاتصال بالـ MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 غير متصل بشكل صحيح!");
    while (1);
  }
  
  // معايرة الـ MPU6050
  calibrateGyroscope();
}

void loop() {
  // قراءة الدوران بزاوية 90 درجة يمين أو شمال
  /*if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'r') {
      rotate90DegreesRight();
    } else if (command == 'l') {
      rotate90DegreesLeft();
    }
  }*/



}

void calibrateGyroscope() {
   
  int readings = 200;
  long sumZ = 0;
  
  for (int i = 0; i < readings; i++) {
    int16_t gyroX, gyroY, gyroZ;
   mpu.getRotation(gyroX, gyroY, gyroZ);
    sumZ += gyroZ;
    delay(3); // ندي وقت للجيروسكوب يثبت
  }
  
  // نحسب المتوسط كقيمة مرجعية للميلان في الوضع الثابت
  gyroOffsetZ = sumZ / readings;
}


// قراءة بيانات الزاوية Yaw (المحور Z)
float getYaw() {
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);  // قراءة بيانات الجيروسكوب
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;  // الوقت بين القراءات
  previousTime = currentTime;
  
  float deltaYaw = ((gz - gyroOffsetZ) / 131.0) * deltaTime;  // تحويل إلى الدرجات في الثانية
  yaw += deltaYaw;
  return yaw;
}

// لف الروبوت بزاوية 90 درجة لليمين
void rotate90DegreesRight() {
  yaw = 0;  // إعادة تعيين الزاوية
  previousTime = millis();  // تعيين الوقت الحالي
  while (yaw < 90) {
    getYaw();
   // Serial.print("Yaw: ");
   // Serial.println(yaw);
    turnRight(100);  // ضبط سرعة الدوران حسب الحاجة
  }
  stopMoving();  // توقف بعد الوصول إلى 90 درجة
}

// لف الروبوت بزاوية 90 درجة لليسار
void rotate90DegreesLeft() {
  yaw = 0;  // إعادة تعيين الزاوية
  previousTime = millis();  // تعيين الوقت الحالي
  while (yaw > -90) {
    getYaw();
  //  Serial.print("Yaw: ");
    //Serial.println(yaw);
    turnLeft(100);  // ضبط سرعة الدوران حسب الحاجة
  }
  stopMoving();  // توقف بعد الوصول إلى -90 درجة
}
void moveForward(int speed)
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(motorRightSpeed, speed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(motorLeftSpeed, speed);

}

// تحريك الموتورات لليمين
void turnRight(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(motorRightSpeed, 0);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(motorLeftSpeed, speed);
}

// تحريك الموتورات لليسار
void turnLeft(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(motorRightSpeed, speed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(motorLeftSpeed, 0);
}

// إيقاف الموتورات
void stopMoving() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(motorRightSpeed, 0);
  analogWrite(motorLeftSpeed, 0);
}

// إعداد المخارج الخاصة بالموتورات
void setupMotors() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(motorRightSpeed, OUTPUT);
  pinMode(motorLeftSpeed, OUTPUT);
}



