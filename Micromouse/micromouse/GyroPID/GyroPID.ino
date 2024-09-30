#include <Wire.h>
#include <MPU6050.h>

MPU6050 gyro;

// متغيرات خاصة بالتحكم في المحركات
const int leftMotorPin = 9;  // دبوس التحكم في المحرك الأيسر
const int rightMotorPin = 10; // دبوس التحكم في المحرك الأيمن
int baseSpeed = 100;  // السرعة الأساسية للمحرك

// المتغيرات الخاصة بـ PID
float kp = 2.0;  // عامل التناسب
float ki = 0.5;  // عامل التكامل
float kd = 0.1;  // عامل التفاضل

float previousError = 0;
float integral = 0;

// متغيرات خاصة بالوقت
long lastTime = 0;

// متغيرات زاوية الدوران
float yaw = 0;
float targetYaw = 0;
float gyroZOffset = 0;

void setup() {
  // تهيئة الاتصال التسلسلي
  Serial.begin(9600);

  // تهيئة الجيروسكوب
  Wire.begin();
  gyro.initialize();

  // التحقق من حالة الجيروسكوب
  if (!gyro.testConnection()) {
    Serial.println("فشل الاتصال بالجيروسكوب!");
    while (1);
  }

  // معايرة الجيروسكوب
  calibrateGyro();
  
  // تهيئة دبابيس المحركات
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);
  
  lastTime = millis();  // تهيئة الوقت
}

void loop() {
  // المشي بخط مستقيم
  moveStraight();

  // مثال على الدوران 90 درجة يمين بعد المشي
  delay(2000);  // انتظر لبعض الوقت
  turn90Degrees(true);  // دوران 90 درجة لليمين

  // المشي بخط مستقيم مرة أخرى
  delay(2000);  // انتظر لبعض الوقت
  moveStraight();

  // مثال على الدوران 90 درجة يسار
  delay(2000);
  turn90Degrees(false);  // دوران 90 درجة لليسار
}

// دالة لتحريك الروبوت بخط مستقيم
void moveStraight() {
  updateYaw();  // تحديث زاوية الدوران

  // حساب الخطأ (فرق زاوية الدوران الحالية عن المستهدفة)
  float error = yaw - targetYaw;

  // استخدام PID لتصحيح الانحراف
  if (abs(error) > 1) {  // هامش ±1 درجة
    long currentTime = millis();
    long elapsedTime = currentTime - lastTime;

    int correction = calculatePID(error, elapsedTime);

    // تعديل السرعة حسب قيمة التصحيح
    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    // تشغيل المحركات
    moveMotors(leftSpeed, rightSpeed);

    lastTime = currentTime;  // تحديث الوقت
  } else {
    // تحرك بسرعة ثابتة إذا كان الانحراف بسيط
    moveMotors(baseSpeed, baseSpeed);
  }
}

// دالة للدوران 90 درجة
void turn90Degrees(bool turnRight) {
  // تعيين الزاوية المستهدفة بناءً على الاتجاه
  if (turnRight) {
    targetYaw += 90;
  } else {
    targetYaw -= 90;
  }

  // إعادة تعيين متغيرات PID
  integral = 0;
  previousError = 0;

  // الاستمرار في الدوران حتى الوصول إلى الزاوية المستهدفة
  while (abs(yaw - targetYaw) > 1) {
    updateYaw();  // تحديث زاوية الدوران

    float error = yaw - targetYaw;
    long currentTime = millis();
    long elapsedTime = currentTime - lastTime;

    int correction = calculatePID(error, elapsedTime);

    // تحريك المحركات بناءً على الاتجاه
    if (turnRight) {
      moveMotors(baseSpeed - correction, -(baseSpeed - correction));  // دوران لليمين
    } else {
      moveMotors(-(baseSpeed - correction), baseSpeed - correction);  // دوران لليسار
    }

    lastTime = currentTime;  // تحديث الوقت
  }

  // إيقاف المحركات بعد الانتهاء من الدوران
  stopMotors();
}

// دالة لحساب PID
int calculatePID(float error, long elapsedTime) {
  integral += error * elapsedTime;
  float derivative = (error - previousError) / elapsedTime;

  int pid = (kp * error) + (ki * integral) + (kd * derivative);

  previousError = error;

  return pid;
}

// دالة لتحريك المحركات
void moveMotors(int leftSpeed, int rightSpeed) {
  // تقييد السرعة بين -255 و 255
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // التحكم في المحركات
  analogWrite(leftMotorPin, leftSpeed);
  analogWrite(rightMotorPin, rightSpeed);
}

// دالة لإيقاف المحركات
void stopMotors() {
  analogWrite(leftMotorPin, 0);
  analogWrite(rightMotorPin, 0);
}

// دالة لتحديث زاوية الدوران Yaw
void updateYaw() {
  int16_t gx, gy, gz;
  long currentTime = millis();
  long elapsedTime = currentTime - lastTime;

  gyro.getRotation(&gx, &gy, &gz);
  float rateZ = (gz - gyroZOffset) / 131.0;  // قراءة معدل الدوران حول المحور Z

  yaw += rateZ * (elapsedTime / 1000.0);  // حساب زاوية الدوران
  lastTime = currentTime;

  Serial.print("Yaw: ");
  Serial.println(yaw);
}

// دالة لمعايرة الجيروسكوب
void calibrateGyro() {
  int16_t gx, gy, gz;
  long sum = 0;

  for (int i = 0; i < 200; i++) {
    gyro.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(3);
  }

  gyroZOffset = sum / 200.0;  // تعيين الإزاحة
}
