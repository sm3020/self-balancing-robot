#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

// Motor driver pins
#define motorA_IN1 4
#define motorA_IN2 5
#define motorA_ENA 6

#define motorB_IN3 7
#define motorB_IN4 8
#define motorB_ENB 9

MPU6050 mpu;

double setpoint = 0.2;
double input;
double output;

double kp = 67;
double ki = 0.8;
double kd = 1.8;

double angleOffset = 0;
double gyroOffset = 0;

PID balancePID(&input, &output, &setpoint, kp, ki, kd, REVERSE);

// Run controller at 100 Hz
unsigned long lastTime = 0;
const int sampleTime = 10;


void setup() {

  Serial.begin(115200);
  Wire.begin();

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  calibrateSensor();

  pinMode(motorA_IN1, OUTPUT);
  pinMode(motorA_IN2, OUTPUT);
  pinMode(motorA_ENA, OUTPUT);
  pinMode(motorB_IN3, OUTPUT);
  pinMode(motorB_IN4, OUTPUT);
  pinMode(motorB_ENB, OUTPUT);

  balancePID.SetMode(AUTOMATIC);
  balancePID.SetSampleTime(sampleTime);
  balancePID.SetOutputLimits(-255, 255);

  delay(2000);
}


double getBalanceAngle() {

  static double gyroAngle = 0;
  static unsigned long prevTime = millis();

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  double accelAngle = atan2(ax, az) * 180.0 / PI;
  accelAngle -= angleOffset;

  unsigned long currentTime = millis();
  double dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // Remove gyro bias & convert to deg/s
  double gyroRate = (gy - gyroOffset) / 131.0;
  gyroAngle += gyroRate * dt;

  // Gyro is smoother short-term; accelerometer corrects drift
  double filteredAngle =
      0.95 * gyroAngle + 0.05 * accelAngle;

  gyroAngle = filteredAngle;

  return filteredAngle;
}


void setMotorSpeeds(int leftSpeed, int rightSpeed) {

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed >= 0) {
    digitalWrite(motorA_IN1, HIGH);
    digitalWrite(motorA_IN2, LOW);
    analogWrite(motorA_ENA, leftSpeed);
  } else {
    digitalWrite(motorA_IN1, LOW);
    digitalWrite(motorA_IN2, HIGH);
    analogWrite(motorA_ENA, abs(leftSpeed));
  }

  if (rightSpeed >= 0) {
    digitalWrite(motorB_IN3, HIGH);
    digitalWrite(motorB_IN4, LOW);
    analogWrite(motorB_ENB, rightSpeed);
  } else {
    digitalWrite(motorB_IN3, LOW);
    digitalWrite(motorB_IN4, HIGH);
    analogWrite(motorB_ENB, abs(rightSpeed));
  }
}


void stopMotors() {
  analogWrite(motorA_ENA, 0);
  analogWrite(motorB_ENB, 0);
}


void loop() {

  unsigned long currentTime = millis();

  if (currentTime - lastTime >= sampleTime) {
    lastTime = currentTime;

    input = getBalanceAngle();

    // Stop trying to balance once the robot is beyond recovery
    if (abs(input) > 45) {
      stopMotors();
      Serial.println("Robot fallen");
      return;
    }

    balancePID.Compute();

    int motorSpeed = (int)output;
    setMotorSpeeds(motorSpeed, motorSpeed);

    Serial.print("Angle: ");
    Serial.print(input);
    Serial.print(" | Output: ");
    Serial.println(output);
  }
}


void calibrateSensor() {

  double angleSum = 0;
  long gyroSum = 0;
  const int samples = 1000;

  Serial.println("Hold robot upright for calibration");

  for (int i = 0; i < samples; i++) {

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    angleSum += atan2(ax, az) * 180.0 / PI;
    gyroSum += gy;

    delay(3);
  }

  angleOffset = angleSum / samples;
  gyroOffset = gyroSum / samples;
}
