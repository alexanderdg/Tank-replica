#ifndef POWER_H
#define POWER_H

#include <Arduino.h>

#define DisablePowerPin 28
#define FPVPowerPin 30
#define EnChargePin1 36
#define EnChargePin2 35
#define I2Cadress 0x40
#define CC1 A6
#define CC2 A7

#define VRxL A3
#define VRyL A2
#define PowerOffTime 10

class Power {
  public:
    Power();
    int initPower(void);
    void disablePower(void);
    void enableFPVPower(void);
    void disableFPVPower(void);
    void setChargeRate1A5(bool);
    void setChargeRate2A5(bool);

    float readCurrent(void);
    float readVoltage(void);
    float readPower(void);
    int readCapacity(void);

    bool checkForCharger(void);
    void checkForPowerdown(void);

  private:
    bool writeI2CByte(int reg, int MSBvalue, int LSBvalue);
    bool readI2CByte(int reg, int * MSBvalue, int * LSBvalue);
    int chargeRate = 0;
    long lastActivityTime = 0;


  
};

#endif
