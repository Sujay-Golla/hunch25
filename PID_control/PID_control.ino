#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Servo.h>

Servo Servo1;
MPU6050 mpu6050(Wire);

const int servoPin = 9;
long timer = 0;

// **Tuned PID Constants**
float Kp = 0.9;    // Proportional Gain (lowered for stability)
float Ki = 0.0001;  // Integral Gain (reduced to prevent windup)
float Kd = 1.2;    // Derivative Gain (increased for damping)

// **PID Variables**
float previousError = 0;
float integral = 0;
const float I_max = 20;  // Limit integral accumulation to prevent windup
unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  mpu6050.setGyroOffsets(0, 0, 0);

  Servo1.attach(servoPin);
}

void loop() {
  mpu6050.update();

  // Read angles
  float accAngleX = mpu6050.getAccAngleX();
  float gyroAngleX = mpu6050.getGyroAngleX();

  // **Complementary filter** (to reduce noise)
  float filteredAngle = 0.98 * gyroAngleX + 0.02 * accAngleX;

  // **PID Calculation**
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;  // Convert ms to seconds
  lastTime = currentTime;

  float error = 0 - filteredAngle;  // Target angle = 0 (balanced position)

  // **Anti-Windup: Limit Integral Accumulation**
  if (abs(error) < 20) {  // Only integrate when error is within a reasonable range
    integral += error * deltaTime;
    integral = constrain(integral, -I_max, I_max);
  }

  float derivative = (error - previousError) / deltaTime;
  previousError = error;

  float pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // **Reverse PID Output to Correctly Stabilize Movement**
  pidOutput = -pidOutput;

  // **Clamp PID Output to Prevent Overshoot**
  pidOutput = constrain(pidOutput, -90, 90);

  // **Map to Servo Range (0° to 180°)**
  int mappedAngle = map(pidOutput, -90, 90, 0, 180);
  mappedAngle = constrain(mappedAngle, 0, 180);

  // Move the Servo
  Servo1.write(mappedAngle);

  // **Debug Output to Monitor PID Behavior**
  Serial.print("Filtered Angle: "); Serial.print(filteredAngle);
  Serial.print("\tPID Output: "); Serial.print(pidOutput);
  Serial.print("\tMapped Servo Angle: "); Serial.println(mappedAngle);

  delay(30);
}
