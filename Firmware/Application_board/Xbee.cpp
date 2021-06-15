#include "Xbee.h"
#include <Arduino.h>


Xbee::Xbee(void) {

}


bool Xbee::initXbee(void) {
  Serial8.begin(115200);
  return true;
}

bool Xbee::readRemote(MotorValues * motorValues, TurretValues * turretValues) {
  bool returnValue = false;
  uint8_t receivedBytes = 0;
  long savedTimestamp = millis();
  Serial8.write('r');
  while (receivedBytes < 2 && (millis() - savedTimestamp) < ConnectionTimeout)
  {
    if (Serial8.available() > 0)
    {
      int received = Serial8.read();
      switch (receivedBytes)
      {
        case 0:
          motorValues -> LeftMotor = map(received, 0, 255, 0, 800);
          break;
        case 1:
          motorValues -> RightMotor = map(received, 0 , 255, 0, 800);
          break;
      }
      receivedBytes ++;
    }
  }
  if((millis() - savedTimestamp) < ConnectionTimeout) returnValue = true;
  return returnValue;
}
