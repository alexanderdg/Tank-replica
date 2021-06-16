

#include <Arduino.h>
#include "Can.h"
#include "Speaker.h"
#include "Xbee.h"


int en_output = 4;
int output1 = 36;
int output2 = 37;
int testpin = 29;

Can can;
Speaker speaker;
Xbee xbee;
MotorValues motorvalues;
TurretValues turretValues;

void setup() {
  can.initCan();
  xbee.initXbee();
  Serial.print("->>");
  Serial.println(can.setAccelerationMotor(15));
  Serial.println(can.setDeaccelerationMotor(8));
  Serial.println(can.setMaxTorque(20));
  speaker.init();
  speaker.playTank();
  pinMode(testpin, OUTPUT);
  //pinMode(en_output, OUTPUT);
  //pinMode(output1, OUTPUT);
  //pinMode(output2, OUTPUT);
}


void loop() {
  //Serial.println("test");
  if (xbee.readRemote(&motorvalues, &turretValues))
  {
    int temp1 = motorvalues.LeftMotor;
    int temp2 = motorvalues.RightMotor;
    int lMotor, rMotor;
    JoyToDiff(temp1, temp2, &lMotor, &rMotor);
    can.setRightMotorSpeed(rMotor);
    can.setLeftMotorSpeed(lMotor);
  }
  else
  {
    //Serial.println("Connection problems with the remote");
    //Serial.println(motorvalues.LeftMotor);
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
