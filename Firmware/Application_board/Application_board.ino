#include <Arduino.h>
#include "Can.h"
#include "Speaker.h"
#include "Xbee.h"
#include <LoRaLib.h>


int en_output = 4;
int output1 = 36;
int output2 = 37;
int testpin = 29;

#define DDRIVE_MIN -256 //The minimum value x or y can be.
#define DDRIVE_MAX 255  //The maximum value x or y can be.
#define MOTOR_MIN_PWM 0 //The minimum value the motor output can be.
#define MOTOR_MAX_PWM 800 //The maximum value the motor output can be.
#define LORA_RST 30
#define LORA_CS 10
#define LORA_DIO0 31
#define LORA_DIO1 32

Can can;
Speaker speaker;
Xbee xbee;
MotorValues motorValues;
TurretValues turretValues;
SX1278 lora = new LoRa(LORA_CS, LORA_DIO0, LORA_DIO1, SPI);

volatile bool receiveFlag = false;
volatile bool enableInterrupt = true;
String tempString = "";
String inputString = "";
bool stringComplete = false;
uint8_t VRxL = 127;
uint8_t VRyL = 127;
int lastDriveMode = 0;

long savedTimestamp = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH);
  can.initCan();
  xbee.initXbee();
  Serial.print("->>");
  Serial.println(can.setAccelerationMotor(12));
  Serial.println(can.setDeaccelerationMotor(8));
  Serial.println(can.setMaxTorque(25));
  speaker.init();
  speaker.playTank();
  pinMode(testpin, OUTPUT);
  //pinMode(en_output, OUTPUT);
  //pinMode(output1, OUTPUT);
  //pinMode(output2, OUTPUT);
  can.setRightMotorSpeed(400);
  can.setLeftMotorSpeed(400);
  delay(500);
  Serial.print(F("Initializing SX1278 ... "));
  int state = lora.beginFSK(434.0, 200.0);
  lora.setNodeAddress(0x01);
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
  lora.setDio0Action(setFlag);
  lora.startReceive();
}


void loop() {
  //Serial.println("test");
  /*
  if (stringComplete) {
    Serial.print("Received string: ");
    Serial.print(inputString);
    Serial.println(':');
    char buffer[5];
    inputString.toCharArray(buffer, 5);
    VRxL = buffer[0];
    VRyL = buffer[1];
    int driveMode = buffer[4];
    //Serial.println(buffer[1], DEC);
    stringComplete = false;
    int temp1 = motorvalues.LeftMotor;
    int temp2 = motorvalues.RightMotor;
    int lMotor, rMotor;
    JoyToDiff(VRxL, VRyL, &lMotor, &rMotor);
    //Serial.print(lMotor);
    //Serial.print(":");
    //Serial.println(rMotor);
    can.setRightMotorSpeed(rMotor);
    can.setLeftMotorSpeed(lMotor);
    if (lastDriveMode != driveMode) {
      lastDriveMode = driveMode;
      switch (driveMode)
      {
        case 1:
          can.setAccelerationMotor(50);
          can.setDeaccelerationMotor(30);
          break;
        case 2:
          can.setAccelerationMotor(12);
          can.setDeaccelerationMotor(8);
          break;
        case 3:
          can.setAccelerationMotor(4);
          can.setDeaccelerationMotor(3);
          break;
      }
    }
  }
  */
  if (xbee.checkForNewData())
  {
    xbee.getJoystickData(&motorValues, &turretValues);
    int lMotor, rMotor;
    JoyToDiff(motorValues.LeftMotor, motorValues.RightMotor, &lMotor, &rMotor);
    can.setRightMotorSpeed(rMotor);
    can.setLeftMotorSpeed(lMotor);
    int driveMode = xbee.getDriveMode();
    if (lastDriveMode != driveMode) {
      lastDriveMode = driveMode;
      switch (driveMode)
      {
        case 1:
          can.setAccelerationMotor(50);
          can.setDeaccelerationMotor(30);
          break;
        case 2:
          can.setAccelerationMotor(12);
          can.setDeaccelerationMotor(8);
          break;
        case 3:
          can.setAccelerationMotor(2);
          can.setDeaccelerationMotor(1);
          break;
      }
    }
  }
  if ((millis() - savedTimestamp) > 100)
  {
    float inputVoltage = can.getInputVoltage();
    float currentLeftMotor = can.getCurrentLeftMotor();
    float currentRightMotor = can.getCurrentRightMotor();
    int speedLeftMotor = can.getSpeedLeftMotor();
    int speedRightMotor = can.getSpeedRightMotor();
    uint8_t msbInputVoltage = (uint8_t) inputVoltage;
    uint8_t lsbInputVoltage = (uint8_t) ((inputVoltage - msbInputVoltage) * 100);
    uint8_t msbCurrentLeftMotor = (uint8_t) currentLeftMotor;
    uint8_t lsbCurrentLeftMotor = (uint8_t) ((currentLeftMotor - msbCurrentLeftMotor) * 100);
    uint8_t msbCurrentRightMotor = (uint8_t) currentRightMotor;
    uint8_t lsbCurrentRightMotor = (uint8_t) ((currentRightMotor - msbCurrentRightMotor) * 100);
    uint8_t msbSpeedLeftMotor = 0xFF & (speedLeftMotor >> 8);
    uint8_t lsbSpeedLeftMotor = 0xFF & speedLeftMotor;
    uint8_t msbSpeedRightMotor = 0xFF & (speedRightMotor >> 8);
    uint8_t lsbSpeedRightMotor = 0xFF & speedRightMotor;
    char message[11];
    message[0] = msbInputVoltage;
    message[1] = lsbInputVoltage;
    message[2] = msbCurrentLeftMotor;
    message[3] = lsbCurrentLeftMotor;
    message[4] = msbCurrentRightMotor;
    message[5] = lsbCurrentRightMotor;
    message[6] = msbSpeedLeftMotor;
    message[7] = lsbSpeedLeftMotor;
    message[8] = msbSpeedRightMotor;
    message[9] = lsbSpeedRightMotor;
    message[10] = ':';
    for (int i = 0; i < 11; i ++) {
      Serial8.print(message[i]);
    }
    savedTimestamp = millis();
  }
  //Serial.println(can.getSpeedRightMotor());
}

void serialEvent8() {
  xbee.readXbeeData();
}

void setFlag(void) {
  // check if the interrupt is enabled
  if (!enableInterrupt) {
    return;
  }
  receiveFlag = true;
}

void JoyToDiff(int x, int y, int *lMotor, int *rMotor) {
  int nJoyX = map(x, 0, 255, -128, 127);
  int nJoyY = map(y, 0, 255, -128, 127);
  float fPivYLimit = 32.0;
  int     nMotMixL;
  int     nMotMixR;
  float   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
  float   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
  int     nPivSpeed;      // Pivot Speed                          (-128..+127)
  float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )
  if (nJoyY >= 0) {
    // Forward
    nMotPremixL = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
    nMotPremixR = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
  } else {
    // Reverse
    nMotPremixL = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
    nMotPremixR = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
  }
  nMotPremixL = nMotPremixL * nJoyY / 128.0;
  nMotPremixR = nMotPremixR * nJoyY / 128.0;
  nPivSpeed = nJoyX;
  fPivScale = (abs(nJoyY) > fPivYLimit) ? 0.0 : (1.0 - abs(nJoyY) / fPivYLimit);

  // Calculate final mix of Drive and Pivot
  nMotMixL = (1.0 - fPivScale) * nMotPremixL + fPivScale * ( nPivSpeed);
  nMotMixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);
  * lMotor = map(nMotMixL, -128, 127, 0, 800);
  * rMotor = map(nMotMixR, -128, 127, 800, 0);
}


void CalculateTankDrive(float x, float y, int *lMotor, int *rMotor)
{
  float rawLeft;
  float rawRight;
  float RawLeft;
  float RawRight;


  // first Compute the angle in deg
  // First hypotenuse
  float z = sqrt(x * x + y * y);

  // angle in radians
  float rad = acos(abs(x) / z);

  // Cataer for NaN values
  if (isnan(rad) == true) {
    rad = 0;
  }

  // and in degrees
  float angle = rad * 180 / PI;

  // Now angle indicates the measure of turn
  // Along a straight line, with an angle o, the turn co-efficient is same
  // this applies for angles between 0-90, with angle 0 the co-eff is -1
  // with angle 45, the co-efficient is 0 and with angle 90, it is 1

  float tcoeff = -1 + (angle / 90) * 2;
  float turn = tcoeff * abs(abs(y) - abs(x));
  turn = round(turn * 100) / 100;

  // And max of y or x is the movement
  float mov = max(abs(y), abs(x));

  // First and third quadrant
  if ((x >= 0 && y >= 0) || (x < 0 && y < 0))
  {
    rawLeft = mov; rawRight = turn;
  }
  else
  {
    rawRight = mov; rawLeft = turn;
  }

  // Reverse polarity
  if (y < 0) {
    rawLeft = 0 - rawLeft;
    rawRight = 0 - rawRight;
  }

  // Update the values
  RawLeft = rawLeft;
  RawRight = rawRight;

  // Map the values onto the defined rang
  * lMotor = map(rawLeft, DDRIVE_MIN, DDRIVE_MAX, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
  * rMotor = map(rawRight, DDRIVE_MIN, DDRIVE_MAX, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
}
/*
  void serialEvent8() {
  while (Serial8.available()) {
    int speed = Serial8.read();
    int c_speed = map(speed, 0, 255, 0 , 800);
    can.setRightMotorSpeed(c_speed);
    can.setLeftMotorSpeed(c_speed);
    //Serial.println("-------------------");
    //Serial.println(speed);
    //Serial.println(c_speed);
    //String temp = Serial8.readString();
    //Serial8.flush();
  }
  }
*/
