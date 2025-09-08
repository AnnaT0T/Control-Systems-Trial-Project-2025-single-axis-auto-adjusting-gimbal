#include <Servo.h>
#include <Wire.h>

int servoPin = 2;
int servoPos = 90;
int desiredAngle = 90;
int offsetAngle = 0;
Servo myServo;
float AccX, AccY, AccZ;

void gyro_signals(void) {
// collect x, y, z acceleration data from MPU6050
  // all necessary?
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
  // initialize MPU6050
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  // initialize servo
  myServo.attach(servoPin);
  // initialize serial monitor
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  gyro_signals(); // collect x, y, z acceleration data from MPU6050

  if (Serial.available() != 0)
  // check if the user inputted anything into serial monitor, if so, read angle
  {
    offsetAngle = Serial.parseInt();
  }
  desiredAngle = 90 + offsetAngle; // adjust default desired angle (90 deg, pointing up) to account for offset angle

  // adjust servo angle so that it's always pointing towards the desired angle
  // determine if add/subtract adjustment angle depending on if servo is currently pointing to the left/right of x-axis
  if (AccX > 0) {
    servoPos = desiredAngle - (desiredAngle - AccY * desiredAngle);
  }
  if (AccX < 0) {
    servoPos = desiredAngle + (desiredAngle - AccY * desiredAngle);
  }

  // ensure code is not pushing servo to adjust beyond its limits (range: 0 to 180 deg)
  if (servoPos < 0) {
    servoPos = 0;
  }
  if (servoPos > 180) {
    servoPos = 180;
  }

  // print servo angle, offset angle, x, y, z acceleration to serial monitor
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
  myServo.write(servoPos); // set servo to calculated angle to make it point towards desired angle
  delay(50); // if too fast, everything moves too quick, issues like serial monitor displaying random characters
}


