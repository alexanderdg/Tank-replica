#include "Xbee.h"
#include <Arduino.h>


Xbee::Xbee(void) {

}


bool Xbee::initXbee(void) {
  Serial8.begin(115200);
  return true;
}


bool Xbee::readXbeeData(void) {
  while (Serial8.available()) {
    char inChar = (char) Serial8.read();
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
    if (inputBufferIndexStored == 5)
    {
      VRxL = inputBufferStored[0];
      VRyL = inputBufferStored[1];
      VRxR = inputBufferStored[2];
      VRyR = inputBufferStored[3];
      driveMode = inputBufferStored[4];
      stringComplete = false;
      newData = true;
    }
    else {
      stringComplete = false;
    }
  }
  return true;
}


bool Xbee::checkForNewData(void) {
  bool returnvalue = newData;
  newData = false;
  return returnvalue;
}


void Xbee::getJoystickData(MotorValues * motorValues, TurretValues * turretValues) {
   motorValues -> LeftMotor = VRxL;
   motorValues -> RightMotor = VRyL;
   turretValues -> xValue = VRxR;
   turretValues -> yValue = VRyR;
}


int Xbee::getDriveMode(void) {
    return driveMode;
}
