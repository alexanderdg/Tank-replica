#ifndef CAN_H
#define CAN_H

#define MotorControllerID 1

#include<Arduino.h>
#include <FlexCAN_T4.h>

class Can
{
  public:
    Can(void);
    bool initCan(void);
    //Motorcontroller methods
    bool coastBrake(void);
    bool dynamicBrake(void);
    bool regenerativeBrake(void);
    bool setLeftMotorSpeed(int speed);
    bool setRightMotorSpeed(int speed);
    bool setLeftRightMotorSpeed(int speedLeft, int speedRight);
    bool setAccelerationMotor(int acceleration);
    bool setDeaccelerationMotor(int deacceleration);
    bool setMaxTorque(int torque);
    bool resetFaults();
    float getInputVoltage(void);
    float getCurrentLeftMotor(void);
    float getCurrentRightMotor(void);
    int getSpeedLeftMotor(void);
    int getSpeedRightMotor(void);
    int getStatus(void);
        
    
  private:
    bool sendFrame(CAN_message_t msg);
    bool requestFrame(CAN_message_t msg);
    void clearBuffer(void);
    CAN_message_t canMessage;
    static Can* CanS;
    bool received = false;
    void checkForEvents(void);
    static void printCanFrame(const CAN_message_t msg);
    static void canReceiveEvent(const CAN_message_t &msg);
    
    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Canbus;
};


#endif
