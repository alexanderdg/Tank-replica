#ifndef XBEE_H
#define XBEE_H

#include <Arduino.h>

#define ConnectionTimeout 20

struct MotorValues {
  int LeftMotor;
  int RightMotor;
};

struct TurretValues {
  uint8_t xValue;
  uint8_t yValue;
};

class Xbee
{

  public:
    Xbee(void);
    bool initXbee(void);
    bool readXbeeData(void);
    bool checkForNewData(void);
    void getJoystickData(MotorValues * motorValues, TurretValues * turretValues);
    int getDriveMode(void);
    

  private:
    
  
    char inputBuffer[20];
    char inputBufferStored[20];
    int inputBufferIndex = 0;
    int inputBufferIndexStored = 0;
    bool stringComplete = false;
    long lastTimeReceivedTimestamp = -1000;

    int VRxL = 127;
    int VRyL = 127;
    int VRxR = 127;
    int VRyR = 127;
    int driveMode = 0;

    bool newData = false;

};


#endif
