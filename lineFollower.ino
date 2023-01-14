#include <QTRSensors.h>

const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;
int m1Speed = 0;
int m2Speed = 0;
// increase kp’s value and see what happens
float kp = 10;
float ki = 0;
float kd = 15;
float p = 1;
float i = 0;
float d = 0;
int error = 0;
int lastError = 0;
const int maxSpeed = 255;
const int minSpeed = -255;
const int baseSpeed = 200;
QTRSensors qtr;
const int sensorCount = 6;
int sensorValues[sensorCount];
int sensors[sensorCount] = { 0, 0, 0, 0, 0, 0 };
void setup() {
  Serial.begin(9600);
  // pinMode setup
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, sensorCount);
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we
                                    // are in calibration mode
  // calibrate the sensor. For maximum grade the line follower should do
  // the movement itself,
  // without human interaction.

  static const long INTERVAL_DURATION = 350;
  static const long CALIBRATION_MOTOR_SPEED = 160;
  const auto start = millis();
  unsigned oldIdx = -1;
  int direction = 1;
  for (uint16_t i = 0; i < 400; i++) {
    const auto intervalIdx = (millis() - start) / INTERVAL_DURATION;
    /*
    if (intervalIdx != oldIdx) {
      oldIdx = intervalIdx;

      if (direction == -1)
        direction = 1;
      else
        direction = -1;

      setMotorSpeed(direction * CALIBRATION_MOTOR_SPEED,
                    direction * CALIBRATION_MOTOR_SPEED * -1);
    }
    */

    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  errorCalculate();
  m1Speed = constrain(m1Speed, 0, maxSpeed);
  m2Speed = constrain(m2Speed, 0, maxSpeed);
  setMotorSpeed(m1Speed, m2Speed);
}
void errorCalculate() {
  int error = map(qtr.readLineBlack(sensorValues), 0, 5000, -255, 255);
  p = error;
  i = i + error;
  d = error - lastError;
  lastError = error;
  int motorSpeed = pidControl(kp, ki, kd);

  m1Speed = baseSpeed;
  m2Speed = baseSpeed;

  if (error < 0) {
    m1Speed += motorSpeed;
  } else if (error > 0) {
    m2Speed -= motorSpeed;
  }
}
// calculate PID value based on error, kp, kd, ki, p, i and d.
int pidControl(float kp, float ki, float kd) {  // TODO
  int motorSpeed = kp * p + ki * i + kd * d;
  return motorSpeed;
}

void setMotorSpeed(int motor1Speed, int motor2Speed) {


  if (motor1Speed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  } else {
    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }
    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }
  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  } else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }
    if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    }
  }
}
