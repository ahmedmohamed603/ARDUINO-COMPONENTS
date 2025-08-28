#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

MPU6050 mpu;

unsigned long lastTime;
float dt;


// Roll, Pitch, Yaw intalize

float angleX = 0, angleY = 0, angleZ = 0; 

//rates intialize
float gyroXrate, gyroYrate, gyroZrate;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

//MPU connection 
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  lastTime = millis();
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Accelerometer angles
float accXangle = atan2(ay, az) * 180 / PI;
float accYangle = 0;

float denom = sqrt((long)ay * ay + (long)az * az);
if (denom != 0) {
  accYangle = atan2(-ax, denom) * 180 / PI;
} 
  //rates (deg/sec)
  gyroXrate = gx / 131.0;
  gyroYrate = gy / 131.0;
  gyroZrate = gz / 131.0;

  //Complementary filter (Roll & Pitch)
  angleX = 0.98 * (angleX + gyroXrate * dt) + 0.02 * accXangle;
  angleY = 0.98 * (angleY + gyroYrate * dt) + 0.02 * accYangle;

  // Yaw from gyro only (not most acurate)
  angleZ += gyroZrate * dt;

  //results
  Serial.print("Roll = "); Serial.print(angleX);
  Serial.print(" | Pitch = "); Serial.print(angleY);
  Serial.print(" | Yaw = "); Serial.println(angleZ);

  delay(10);

//MPU reconnecting
   while(!mpu.testConnection()){
Serial.println("MPU6050 connection failed Reconnecting");
mpu.initialize();
}
}
