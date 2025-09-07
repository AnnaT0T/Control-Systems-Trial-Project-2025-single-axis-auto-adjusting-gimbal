#include <Servo.h>
int servoPin = 2;
int servoPos = 0;
Servo myServo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myServo.attach(servoPin);
}

void loop() {
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
}
