#include <Arduino.h>
#include "Xbee.h"

Xbee::Xbee(void) {
  pinMode(XBEE_RESET, OUTPUT);
}

void Xbee::initXbee(void) {
  Serial5.begin(115200);
  enableXbee();
}

bool Xbee::readXbeeData(void) {
  while (Serial5.available()) {
    char inChar = (char)Serial5.read();
    lastTimeReceivedTimestamp = millis();
    if (inChar == ':') {
      for (int i = 0; i < 20; i++) {
        inputBufferStored[i] = inputBuffer[i];
      }
      inputBufferIndexStored = inputBufferIndex;
      inputBufferIndex = 0;
      for (int i = 0; i < 20; i++) {
        inputBuffer[i] = '0';
      }
      stringComplete = true;
    }
    else {
      if (inputBufferIndex < 20)
      {
        inputBuffer[inputBufferIndex] = inChar;
        inputBufferIndex ++;
      }
      else
      {
        inputBufferIndex = 0;
        for (int i = 0; i < 20; i++) {
          inputBuffer[i] = '0';
        }
      }
    }
  }
  if (stringComplete) {
    if (inputBufferIndexStored == 10)
    {
      tankVoltage = inputBufferStored[0] + (inputBufferStored[1] / 100.0);
      tankCurrentLeftMotor = inputBufferStored[2] + (inputBufferStored[3] / 100.0);
      tankCurrentRightMotor = inputBufferStored[4] + (inputBufferStored[5] / 100.0);
      tankSpeedLeftMotor = (inputBufferStored[6] << 8) + inputBufferStored[7];
      tankSpeedRightMotor = (inputBufferStored[8] << 8) + inputBufferStored[9];
      stringComplete = false;
    }
    else {
      stringComplete = false;
    }
  }
  return checkForConnection();
}

bool Xbee::checkForConnection(void) {
  bool returnvalue = false;
  if ((millis() - lastTimeReceivedTimestamp) < 1000)
  {
    returnvalue = true;
  }
  return returnvalue;
}

void Xbee::enableXbee(void) {
  digitalWrite(XBEE_RESET, LOW);
}

void Xbee::disableXbee(void) {
  digitalWrite(XBEE_RESET, HIGH);
}

void Xbee::sleepXbee(void) {
    
}

void Xbee::sendApiFrame(uint8_t type, uint8_t id, uint8_t* data, uint16_t length) {
  size_t frameLength = 1 + 2 + length + 1 + 2;
  uint8_t* frame = new uint8_t[frameLength];

  frame[0] = 0x7E;                          // start delimiter
  frame[1] = ((length + 2) & 0xFF00) >> 8;  // length MSB
  frame[2] = (length + 2) & 0x00FF;         // length LSB
  frame[3] = type;                          // frame type
  frame[4] = id;                            // frame ID
  memcpy(frame + 5, data, length);          // data

  // calculate the checksum
  uint8_t checksum = 0;
  for(size_t i = 3; i < frameLength - 1; i++) {
    checksum += frame[i];
  }
  frame[5 + length] = 0xFF - checksum;

  // send the frame
  for(size_t i = 0; i < frameLength; i++) {
    Serial5.write(frame[i]);
  }
}


//----------------------------------------------------------------------
//---                 Get methods                                    ---
//----------------------------------------------------------------------

float Xbee::getTankVoltage(void) {
  float returnvalue = -1.0;
  if (checkForConnection()) returnvalue = tankVoltage;
  return returnvalue;
}

float Xbee::getTankCurrentLeftMotor(void) {
  float returnvalue = -1.0;
  if (checkForConnection()) returnvalue = tankCurrentLeftMotor;
  return returnvalue;
}

float Xbee::getTankCurrentRightMotor(void) {
  float returnvalue = -1.0;
  if (checkForConnection()) returnvalue = tankCurrentRightMotor;
  return returnvalue;
}

int Xbee::getTankSpeedLeftMotor(void) {
  int returnvalue = -1;
  if (checkForConnection()) returnvalue = tankSpeedLeftMotor;
  return returnvalue;
}

int Xbee::getTankSpeedRightMotor(void) {
  int returnvalue = -1;
  if (checkForConnection()) returnvalue = tankSpeedRightMotor;
  return returnvalue;
}

float Xbee::getTankSpeedLeftMotorMS(void) {
  float returnvalue = -1.0;
  if (getTankSpeedLeftMotor() != -1) {
    int countSecond = getTankSpeedLeftMotor() * 10;
    float RPS = countSecond / (ENCODER_COUNTS_PER_REVOLUTION / 4);
    returnvalue = RPS * perimeterWheel;
  }
  return returnvalue;
}

float Xbee::getTankSpeedRightMotorMS(void) {
  float returnvalue = -1.0;
  if (getTankSpeedRightMotor() != -1) {
    int countSecond = getTankSpeedRightMotor() * 10;
    float RPS = countSecond / (ENCODER_COUNTS_PER_REVOLUTION / 4);
    returnvalue = RPS * perimeterWheel;
  }
  return returnvalue;
}
