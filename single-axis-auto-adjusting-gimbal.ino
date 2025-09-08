#include <Servo.h>
#include <Wire.h>

int servoPin = 2;
int servoPos = 90;
Servo myServo;
float RateRoll, RatePitch, RateYaw;
float RateCalibrationPitch;
int RateCalibrationNumber;
// necessary?
//#include <PulsePosition.h>
//PulsePositionInput ReceiverInput(RISING);
float ReceiverValue[] = {0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber = 0;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};
float DesiredAnglePitch;
float ErrorAnglePitch;

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1/(1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

/*
void read_receiver(void) {
  ChannelNumber = ReceiverInput.available();
  if (ChannelNumber > 0) {
    for (int i = 1; i <= ChannelNumber, i++) {
      ReceiverValue[i-1] = ReceiverInput.read(i);
    }
  }
}*/

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
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;
  AngleRoll = atan(AccY/sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142/180);
  AnglePitch = -atan(AccX/sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142/180);
}

void setup() {
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationPitch += RatePitch;
    delay(1);
  }
  RateCalibrationPitch /= 2000;
  
  // put your setup code here, to run once:
  //Serial.begin(9600);
  myServo.attach(servoPin);
  
  
  //Wire.begin();
  Serial.begin(115200);
  //initialize_MPU6050();
  
}

void loop() {
  gyro_signals();
  RatePitch -= RateCalibrationPitch;
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0]; KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
  DesiredAnglePitch = 0.10 * (ReceiverValue[1] - 1500) + 190;
  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;

  if (servoPos < 0) {
    servoPos = 0;
  }
  if (servoPos > 180) {
    servoPos = 180;
  }

  if (AccX > 0) {
    servoPos = 90 - (90 - AccY * 90);
  }
  if (AccX < 0) {
    servoPos = 90 + (90 - AccY * 90);
  }




  Serial.print("Servo Angle: ");
  Serial.print(servoPos);
  Serial.print(" | Acc X: ");
  Serial.print(AccX);
  Serial.print(" | Acc Y: ");
  Serial.print(AccY);
  Serial.print(" | AccZ: ");
  Serial.println(AccZ);
  myServo.write(servoPos);
  /*
  myServo.write(0);
  delay(1000);
  myServo.write(90);
  delay(1000);
  myServo.write(180);
  delay(1000);*/
  delay(50);
  
  /*
  // put your main code here, to run repeatedly:
  Serial.println("What Angle for the Servo? "); // ask
  while (Serial.available() == 0) 
  // wait (while there is no input data...)
  // finishes while loop once user inputs data
  {

  }
  servoPos = Serial.parseInt(); // read (stores user-specified angle into var)
  Serial.print("Your number: ");
  Serial.println(servoPos);
  myServo.write(servoPos); // sets servo to specified angle
  */
  /*
  read_MPU6050();
  delay(100);
  */
}

/*
void initialize_MPU6050() {
  
  // Activate the MPU-6050
  Wire.beginTransmission(0x68); // Device Address
  Wire.write(0x6B); // Register to write too
  Wire.write(0x00); // Value to write
  Wire.endTransmission();

  // Set Accelerometer sensitivity
  // Wire.write; 2g -> 0x00, 4g -> 0x08, 8g -> 0x10, 16g -> 0x18
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();

  // Set Gyro sensitivity
  // 250 deg/s -> 0x00, 500 deg/s -> 0x08, 1000 deg/s -> 0x10, 2000 deg/s -> 0x18
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}

void read_MPU6050() {
  // Set the Register to read from
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();

  // Request 14 bytes from MPU6050
  Wire.requestFrom(0x68, 14);

  // Read data 6x Acceleration Bytes, 2x Temp Bytes, 6x Gyro Bytes
  //                High Byte            Low Byte
  int16_t acc_x = Wire.read() << 8 | Wire.read();
  int16_t acc_y = Wire.read() << 8 | Wire.read();
  int16_t acc_z = Wire.read() << 8 | Wire.read();
  int16_t temperature = Wire.read() << 8 | Wire.read();
  int16_t gyro_x = Wire.read() << 8 | Wire.read();
  int16_t gyro_y = Wire.read() << 8 | Wire.read();
  int16_t gyro_z = Wire.read() << 8 | Wire.read();

  // Acceleration Conversion (into g)
  double aX = acc_x/8192.0;
  double aY = acc_y/8192.0;
  double aZ = acc_z/8192.0;

  // Temperature Conversion (into C)
  float temp = (temperature/340.0) + 36.53;

  // Gyroscope Conversion (into degrees/second)
  double gX = gyro_x/65.5;
  double gY = gyro_y/65.5;
  double gZ = gyro_z/65.5;

  Serial.print(gX);
  Serial.print(" | ");
  Serial.print(gY);
  Serial.print(" | ");
  Serial.println(gZ);
}*/

