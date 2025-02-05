#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Servo.h>

Servo Servo1;
MPU6050 mpu6050(Wire);

const int servoPin = 9;

// PID Gains
float Kp = 2.0;   // Proportional gain (Adjust as needed)
float Ki = 0.005;  // Integral gain (Prevents steady-state error)
float Kd = 0.01;   // Derivative gain (Dampens oscillations)

float previousError = 0;
float integral = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);  
  Serial.println("MPU6050 initialized.");

  Servo1.attach(servoPin);  
}

void loop() {
  mpu6050.update();

  float accAngleX = mpu6050.getAccAngleX();
  float gyroAngleX = mpu6050.getGyroAngleX();

  float filteredAngle = 0.98 * gyroAngleX + 0.02 * accAngleX;

  // Time difference for derivative term
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;  
  lastTime = currentTime;

  // PID Control
  float error = 0 - filteredAngle;  // Target = 0 (trying to balance)
  integral += error * deltaTime;  // Accumulate error
  float derivative = (error - previousError) / deltaTime;  
  previousError = error;

  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Map output to servo range
  int servoAngle = map(output, -30, 30, 0, 180);
  servoAngle = constrain(servoAngle, 0, 180);

  Servo1.write(servoAngle);

  // Debugging
  Serial.print("Filtered Angle: "); Serial.print(filteredAngle);
  Serial.print("\tPID Output: "); Serial.print(output);
  Serial.print("\tServo Angle: "); Serial.println(servoAngle);

  delay(50);  // Small delay for smooth updates
}
