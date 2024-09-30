#include <Wire.h>
#include <MPU6050.h>

MPU6050 gyro;  // إنشاء كائن MPU6050

float offsetX = 0, offsetY = 0, offsetZ = 0;  // متغيرات للأوفست
float angleX = 0, angleY = 0, angleZ = 0;    // متغيرات لتخزين الزوايا
unsigned long lastTime = 0;  // لتتبع الوقت
bool isCalibrated = false;   // فلاغ لضبط الأوفست مرة واحدة فقط

void setup() {
  Serial.begin(115200);  // بدء الاتصال التسلسلي
  Wire.begin(21, 22);    // تهيئة I2C مع SDA = GPIO 21 و SCL = GPIO 22

  gyro.initialize();  // تهيئة الجيروسكوب

  if (gyro.testConnection()) {
    Serial.println("MPU6050 جاهز للعمل!");
  } else {
    Serial.println("فشل في الاتصال بـ MPU6050!");
  }

  lastTime = millis();  // الحصول على الوقت الحالي
}

void loop() {
  if (!isCalibrated) {
    // ضبط الأوفست مرة واحدة فقط عند بدء التشغيل
    calibrateGyro();
    isCalibrated = true;  // تغيير الفلاغ لتجنب إعادة الضبط
  }

  int16_t gx, gy, gz;  // متغيرات لاستقبال القيم من المحاور
  gyro.getRotation(&gx, &gy, &gz);  // الحصول على قيم الدوران لكل محور

  // تعويض القيم بالأوفست
  float gX = (gx / 131.0) - offsetX;
  float gY = (gy / 131.0) - offsetY;
  float gZ = (gz / 131.0) - offsetZ;

  // حساب الوقت المنقضي
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;  // تحويل الزمن إلى ثواني
  lastTime = currentTime;

  // حساب التغير في الزوايا (الزاوية = السرعة الزاوية * الزمن)
  angleX += gX * deltaTime;
  angleY += gY * deltaTime;
  angleZ += gZ * deltaTime;

  // طباعة الزوايا
  Serial.print("الزاوية حول المحور X: ");
  Serial.print(angleX);
  Serial.print(" | ");

  Serial.print("الزاوية حول المحور Y: ");
  Serial.print(angleY);
  Serial.print(" | ");

  Serial.print("الزاوية حول المحور Z: ");
  Serial.println(angleZ);

  delay(100);  // تأخير بسيط بين كل قراءة
}

// دالة لضبط الأوفست عبر حساب متوسط القيم أثناء الثبات
void calibrateGyro() {
  int16_t gx, gy, gz;
  long sumX = 0, sumY = 0, sumZ = 0;
  int samples = 1000;

  Serial.println("جاري ضبط الأوفست، من فضلك ابقِ الجهاز ثابتاً...");

  for (int i = 0; i < samples; i++) {
    gyro.getRotation(&gx, &gy, &gz);  // قراءة القيم
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(2);  // تأخير بسيط بين كل قراءة
  }

  // حساب متوسط القيم
  offsetX = (sumX / samples) / 131.0;
  offsetY = (sumY / samples) / 131.0;
  offsetZ = (sumZ / samples) / 131.0;

  Serial.println("تم ضبط الأوفست بنجاح!");
}

