#include <Servo.h>
#include <Wire.h>

int servoPin = 2;
int servoPos = 90;
int desiredAngle = 90;
int offsetAngle = 0;
Servo myServo;
float AccX, AccY, AccZ;

void gyro_signals(void) {
  // necessary?
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  myServo.attach(servoPin);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  gyro_signals();

  if (Serial.available() != 0)
  {
    offsetAngle = Serial.parseInt();
  }
  desiredAngle = 90 + offsetAngle;

  if (AccX > 0) {
    servoPos = desiredAngle - (desiredAngle - AccY * desiredAngle);
  }
  if (AccX < 0) {
    servoPos = desiredAngle + (desiredAngle - AccY * desiredAngle);
  }

  if (servoPos < 0) {
    servoPos = 0;
  }
  if (servoPos > 180) {
    servoPos = 180;
  }

  Serial.print("Servo Angle: ");
  Serial.print(servoPos);
  Serial.print(" | Offset Angle: ");
  Serial.print(offsetAngle);
  Serial.print(" | Acc X: ");
  Serial.print(AccX);
  Serial.print(" | Acc Y: ");
  Serial.print(AccY);
  Serial.print(" | AccZ: ");
  Serial.println(AccZ);
  myServo.write(servoPos);
  delay(50);
}


