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
    bool readRemote(MotorValues * motorValues, TurretValues * turretValues);

  private:
    

};


#endif
