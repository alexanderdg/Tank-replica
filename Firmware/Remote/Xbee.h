#ifndef XBEE_H
#define XBEE_H

#include <Arduino.h>

#define DIAMETER_WHEEL 120
#define DISTANCE_BETWEEN_WHEELS 280
#define ENCODER_COUNTS_PER_REVOLUTION 3591.84
#define XBEE_RESET 2

class Xbee {
  public:
    Xbee();
    void initXbee(void);
    bool readXbeeData(void);
    bool checkForConnection(void);
    void enableXbee(void);
    void disableXbee(void);
    void sleepXbee(void);

    float getTankVoltage(void);
    float getTankCurrentLeftMotor(void);
    float getTankCurrentRightMotor(void);
    int getTankSpeedLeftMotor(void);
    float getTankSpeedLeftMotorMS(void);
    int getTankSpeedRightMotor(void);
    float getTankSpeedRightMotorMS(void);

  private:
    void sendApiFrame(uint8_t type, uint8_t id, uint8_t* data, uint16_t length);
  
    String tempString = "";
    String inputString = "";
    char inputBuffer[20];
    char inputBufferStored[20];
    int inputBufferIndex = 0;
    int inputBufferIndexStored = 0;
    bool stringComplete = false;

    long lastTimeReceivedTimestamp = -1000;

    float radiusWheel = DIAMETER_WHEEL / 2000.0;
    float perimeterWheel = 2 * PI * radiusWheel;

    //variables for data from tank
    float tankVoltage = 0.0;
    float tankCurrentLeftMotor = 0.0;
    float tankCurrentRightMotor = 0.0;
    float tankSpeedLeftMotor = 0;
    float tankSpeedRightMotor = 0;
};



#endif
