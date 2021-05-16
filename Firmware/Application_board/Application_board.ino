#include <Arduino.h>
#include "Can.h"

Can can;

void setup() {
  can.initCan(); 
  Serial.print("->>");
  Serial.println(can.setAccelerationMotor(100));
  Serial.println(can.setMaxTorque(100));
}


void loop() {
  Serial.println(can.setRightMotorSpeed(300));
  Serial.println(can.getCurrentLeftMotor());
  Serial.println(can.getInputVoltage());
  Serial.println(can.getCurrentRightMotor());
  delay(3000);
  //Serial.println(can.coastBrake());
  //Serial.println(can.setLeftMotorSpeed(200));
  delay(1000);
}
