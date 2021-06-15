#include <Arduino.h>
#include "CAN.h"

Can* Can::CanS = 0;

Can::Can() {
  CanS = this;
}

bool Can::initCan(void) {
  Canbus.begin();
  Canbus.setBaudRate(250000);
  Canbus.setMaxMB(16);
  Canbus.enableFIFO();
  Canbus.enableFIFOInterrupt();
  Canbus.onReceive(Can::canReceiveEvent);
  Canbus.mailboxStatus();
  return true;
}

//-------------------------------- Motorcontroller methods -------------------------------//

bool Can::coastBrake(void) {
  CAN_message_t msg;
  msg.id = MotorControllerID;
  msg.buf[0] = 0;
  msg.len = 1;
  return sendFrame(msg);
}

bool Can::dynamicBrake(void) {
  CAN_message_t msg;
  msg.id = MotorControllerID;
  msg.buf[0] = 1;
  msg.len = 1;
  return sendFrame(msg);
}

bool Can::regenerativeBrake(void) {
  CAN_message_t msg;
  msg.id = MotorControllerID;
  msg.buf[0] = 2;
  msg.len = 1;
  return sendFrame(msg);
}

bool Can::setLeftMotorSpeed(int speed) {
  CAN_message_t msg;
  msg.id = MotorControllerID;
  msg.buf[0] = 5;
  msg.buf[1] = (speed >> 8) & 0xFF;
  msg.buf[2] = speed & 0xFF;
  msg.len = 3;
  return sendFrame(msg);
}

bool Can::setRightMotorSpeed(int speed) {
  CAN_message_t msg;
  msg.id = MotorControllerID;
  msg.buf[0] = 6;
  msg.buf[1] = (speed >> 8) & 0xFF;
  msg.buf[2] = speed & 0xFF;
  msg.len = 3;
  return sendFrame(msg);
}

bool Can::setLeftRightMotorSpeed(int speedLeft, int speedRight) {
  CAN_message_t msg;
  msg.id = MotorControllerID;
  msg.buf[0] = 7;
  msg.buf[1] = (speedLeft >> 8) & 0xFF;
  msg.buf[2] = speedLeft & 0xFF;
  msg.buf[3] = (speedRight >> 8) & 0xFF;
  msg.buf[4] = speedRight & 0xFF;
  msg.len = 3;
  return sendFrame(msg);
}

bool Can::setAccelerationMotor(int acceleration) {
  CAN_message_t msg;
  msg.id = MotorControllerID;
  msg.buf[0] = 10;
  msg.buf[1] = acceleration;
  msg.len = 2;
  return sendFrame(msg);
}

bool Can::setDeaccelerationMotor(int deacceleration) {
  CAN_message_t msg;
  msg.id = MotorControllerID;
  msg.buf[0] = 11;
  msg.buf[1] = deacceleration;
  msg.len = 2;
  return sendFrame(msg);
}

bool Can::setMaxTorque(int torque) {
  CAN_message_t msg;
  msg.id = MotorControllerID;
  msg.buf[0] = 12;
  msg.buf[1] = torque;
  msg.len = 2;
  return sendFrame(msg);
}

float Can::getCurrentLeftMotor(void) {
  float returnvalue = -1.0;
  CAN_message_t msg;
  msg.id = MotorControllerID;
  msg.buf[0] = 101;
  msg.len = 1;
  if (requestFrame(msg)) {
    returnvalue = canMessage.buf[1] + (canMessage.buf[2] / 100.0);
  }
  return returnvalue;
}

float Can::getCurrentRightMotor(void) {
  float returnvalue = -1.0;
  CAN_message_t msg;
  msg.id = MotorControllerID;
  msg.buf[0] = 100;
  msg.len = 1;
  if (requestFrame(msg)) {
    returnvalue = canMessage.buf[1] + (canMessage.buf[2] / 100.0);
  }
  return returnvalue;
}

float Can::getInputVoltage(void) {
  float returnvalue = -1.0;
  CAN_message_t msg;
  msg.id = MotorControllerID;
  msg.buf[0] = 102;
  msg.len = 1;
  if (requestFrame(msg)) {
    returnvalue = canMessage.buf[1] + (canMessage.buf[2] / 100.0);
  }
  return returnvalue;
}

int Can::getSpeedLeftMotor(void) {
  int returnvalue = -1;
  CAN_message_t msg;
  msg.id = MotorControllerID;
  msg.buf[0] = 103;
  msg.len = 1;
  if (requestFrame(msg)) {
    returnvalue = canMessage.buf[1];
  }
  return returnvalue;
}

int Can::getSpeedRightMotor(void) {
  int returnvalue = -1;
  CAN_message_t msg;
  msg.id = MotorControllerID;
  msg.buf[0] = 104;
  msg.len = 1;
  if (requestFrame(msg)) {
    returnvalue = canMessage.buf[1];
  }
  return returnvalue;
}
//----------------------------------------------------------------------------------------//
void Can::checkForEvents(void) {
  Canbus.events();
}

void Can::canReceiveEvent(const CAN_message_t &msg) {
  CanS -> received = true;
  CanS -> canMessage = msg;
}

void Can::clearBuffer(void) {
  checkForEvents();
}

bool Can::requestFrame(CAN_message_t msg) {
  bool returnvalue = false;
  Canbus.write(msg);
  received = false;
  uint32_t timeout = millis();
  while (CanS -> received == false && ((millis() - timeout) < 3)) {
    checkForEvents();
  }
  if (((millis() - timeout) < 3)) returnvalue = true;
  return returnvalue;
}

bool Can::sendFrame(CAN_message_t msg) {
  bool returnvalue = false;
  Canbus.write(msg);
  received = false;
  uint32_t timeout = millis();
  while (CanS -> received == false && ((millis() - timeout) < 3)) {
    checkForEvents();
  }
  if (((millis() - timeout) < 3) && canMessage.buf[1] == 1) returnvalue = true;
  return returnvalue;
  
}

void Can::printCanFrame(const CAN_message_t msg) {
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}
