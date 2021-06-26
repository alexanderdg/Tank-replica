

#include <Arduino.h>
#include "Can.h"
#include "Speaker.h"
#include "Xbee.h"


int en_output = 4;
int output1 = 36;
int output2 = 37;
int testpin = 29;

#define DDRIVE_MIN -256 //The minimum value x or y can be.
#define DDRIVE_MAX 255  //The maximum value x or y can be.
#define MOTOR_MIN_PWM 0 //The minimum value the motor output can be.
#define MOTOR_MAX_PWM 800 //The maximum value the motor output can be.

Can can;
Speaker speaker;
Xbee xbee;
MotorValues motorvalues;
TurretValues turretValues;

void setup() {
  can.initCan();
  xbee.initXbee();
  Serial.print("->>");
  Serial.println(can.setAccelerationMotor(12));
  Serial.println(can.setDeaccelerationMotor(8));
  Serial.println(can.setMaxTorque(25));
  speaker.init();
  speaker.playTank();
  pinMode(testpin, OUTPUT);
  //pinMode(en_output, OUTPUT);
  //pinMode(output1, OUTPUT);
  //pinMode(output2, OUTPUT);
  can.setRightMotorSpeed(400);
  can.setLeftMotorSpeed(400);
}


void loop() {
  //Serial.println("test");
  if (xbee.readRemote(&motorvalues, &turretValues))
  {
    int temp1 = motorvalues.LeftMotor;
    int temp2 = motorvalues.RightMotor;
    int lMotor, rMotor;
    JoyToDiff(temp1, temp2, &lMotor, &rMotor);
    //CalculateTankDrive(temp1, temp2, &lMotor, &rMotor);
    //Serial.print(temp1);
    //Serial.print("\t");
    //Serial.println(temp2);
    can.setRightMotorSpeed(rMotor);
    can.setLeftMotorSpeed(lMotor);
  }

  else
  {
    //Serial.println("Connection problems with the remote");
    //Serial.println(motorvalues.LeftMotor);
  }
  if (millis() > 8000)
  {
    //can.setRightMotorSpeed(800);
    //can.setLeftMotorSpeed(800);
  }
  
    Serial.println("Voltage requested");
    float voltage = can.getInputVoltage();
    Serial.println(voltage);
    if (voltage < 0.0)
    {
    Serial.println("Error detected !!!!!!!!!!!!!!!!!!");
    digitalWrite(testpin, !digitalRead(testpin));
    }
    float current1 = can.getCurrentLeftMotor();
    Serial.println(current1);
    float current2 = can.getCurrentRightMotor();
     Serial.println(current2);
    Serial.println(can.getSpeedLeftMotor());

    Serial.println(can.getSpeedRightMotor());



  delay(18);
}

void JoyToDiff(int x, int y, int *lMotor, int *rMotor) {
  int nJoyX = map(x, 0, 800, -128, 127);
  int nJoyY = map(y, 0, 800, -128, 127);
  float fPivYLimit = 32.0;
  int     nMotMixL;
  int     nMotMixR;
  float   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
  float   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
  int     nPivSpeed;      // Pivot Speed                          (-128..+127)
  float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )
  if (nJoyY >= 0) {
    // Forward
    nMotPremixL = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
    nMotPremixR = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
  } else {
    // Reverse
    nMotPremixL = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
    nMotPremixR = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
  }
  nMotPremixL = nMotPremixL * nJoyY / 128.0;
  nMotPremixR = nMotPremixR * nJoyY / 128.0;
  nPivSpeed = nJoyX;
  fPivScale = (abs(nJoyY) > fPivYLimit) ? 0.0 : (1.0 - abs(nJoyY) / fPivYLimit);

  // Calculate final mix of Drive and Pivot
  nMotMixL = (1.0 - fPivScale) * nMotPremixL + fPivScale * ( nPivSpeed);
  nMotMixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);
  * lMotor = map(nMotMixL, -128, 127, 0, 800);
  * rMotor = map(nMotMixR, -128, 127, 800, 0);
}


void CalculateTankDrive(float x, float y, int *lMotor, int *rMotor)
{
  float rawLeft;
  float rawRight;
  float RawLeft;
  float RawRight;


  // first Compute the angle in deg
  // First hypotenuse
  float z = sqrt(x * x + y * y);

  // angle in radians
  float rad = acos(abs(x) / z);

  // Cataer for NaN values
  if (isnan(rad) == true) {
    rad = 0;
  }

  // and in degrees
  float angle = rad * 180 / PI;

  // Now angle indicates the measure of turn
  // Along a straight line, with an angle o, the turn co-efficient is same
  // this applies for angles between 0-90, with angle 0 the co-eff is -1
  // with angle 45, the co-efficient is 0 and with angle 90, it is 1

  float tcoeff = -1 + (angle / 90) * 2;
  float turn = tcoeff * abs(abs(y) - abs(x));
  turn = round(turn * 100) / 100;

  // And max of y or x is the movement
  float mov = max(abs(y), abs(x));

  // First and third quadrant
  if ((x >= 0 && y >= 0) || (x < 0 && y < 0))
  {
    rawLeft = mov; rawRight = turn;
  }
  else
  {
    rawRight = mov; rawLeft = turn;
  }

  // Reverse polarity
  if (y < 0) {
    rawLeft = 0 - rawLeft;
    rawRight = 0 - rawRight;
  }

  // Update the values
  RawLeft = rawLeft;
  RawRight = rawRight;

  // Map the values onto the defined rang
  * lMotor = map(rawLeft, DDRIVE_MIN, DDRIVE_MAX, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
  * rMotor = map(rawRight, DDRIVE_MIN, DDRIVE_MAX, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
}
/*
  void serialEvent8() {
  while (Serial8.available()) {
    int speed = Serial8.read();
    int c_speed = map(speed, 0, 255, 0 , 800);
    can.setRightMotorSpeed(c_speed);
    can.setLeftMotorSpeed(c_speed);
    //Serial.println("-------------------");
    //Serial.println(speed);
    //Serial.println(c_speed);
    //String temp = Serial8.readString();
    //Serial8.flush();
  }
  }
*/
