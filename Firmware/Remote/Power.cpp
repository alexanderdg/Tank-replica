#include <Arduino.h>
#include "Power.h"
#include <Wire.h>

Power::Power(void) {
  pinMode(DisablePowerPin, OUTPUT);
  pinMode(FPVPowerPin, OUTPUT);
  pinMode(EnChargePin1, OUTPUT);
  pinMode(EnChargePin2, OUTPUT);
}

int Power::initPower(void) {
  Wire1.begin();
  //RST = 1: Activate reset
  //BRNG = 1: FSR of busvoltage at 32V
  //PGA = 0b00: set gain to max
  //BADC 4-2 = 0b111: set Averaging samples to max 128 for the Bus voltage
  int MSBdata = 0xA7;
  //BADC 1 = 1: set Averaging samples to max 128
  //SADC 4-1 = 0b1111: set Averaging samples to max 128 for the shunt voltage
  // Mode 3-1 = 0b111: Set ADC in continous mode for the Shunt and bus voltage
  int LSBdata = 0xFF;
  writeI2CByte(0x00, MSBdata, LSBdata);
  //Program the calibration register with 6826
  MSBdata = 0x6A;
  LSBdata = 0xAA;
  writeI2CByte(0x05, MSBdata, LSBdata);
  return 0;
}

void Power::disablePower(void) {
  Serial.println("Hardware power disable activated");
  digitalWrite(DisablePowerPin, HIGH);
}

void Power::enableFPVPower(void) {
  digitalWrite(FPVPowerPin, HIGH);
}

void Power::disableFPVPower(void) {
  digitalWrite(FPVPowerPin, LOW);
}

void Power::setChargeRate1A5(bool value) {
  if (value) {
    digitalWrite(EnChargePin1, HIGH);
    digitalWrite(EnChargePin2, LOW);
    chargeRate = 1;
  }
  else {
    digitalWrite(EnChargePin1, LOW);
    digitalWrite(EnChargePin2, LOW);
    chargeRate = 0;
  }
}

void Power::setChargeRate2A5(bool value) {
  if (value) {
    digitalWrite(EnChargePin1, HIGH);
    digitalWrite(EnChargePin2, HIGH);
    chargeRate = 2;
  }
  else {
    digitalWrite(EnChargePin1, LOW);
    digitalWrite(EnChargePin2, LOW);
    chargeRate = 0;
  }
}

float Power::readCurrent(void) {
  int MSBdata, LSBdata;
  readI2CByte(0x04, & MSBdata, & LSBdata);
  int temp = (MSBdata << 8) + LSBdata;
  float normalized = temp * 0.00015;
  return normalized;
}

float Power::readVoltage(void) {
  int MSBdata, LSBdata;
  readI2CByte(0x02, & MSBdata, & LSBdata);
  int temp = (MSBdata << 5) + ((LSBdata & 0b11111000) >> 3);
  float normalized = temp * 0.004;
  return normalized;
}

float Power::readPower(void) {
  int MSBdata, LSBdata;
  readI2CByte(0x03, & MSBdata, & LSBdata);
  int temp = (MSBdata << 8) + LSBdata;
  float normalized = temp * 0.003;
  return normalized;
}

int Power::readCapacity(void) {
  float voltage = readVoltage();
  //crude internal resistance cancelation to avoid different batterylevel readings
  if (checkForCharger()) {
    if (chargeRate == 0) voltage = voltage - 0.02;
    else if (chargeRate == 1) voltage = voltage - 0.06;
    else voltage = voltage - 0.09;
  }
  int batterylevel = 0;
  if (voltage > 3.97) batterylevel = 4;
  else if (voltage > 3.85) batterylevel = 3;
  else if (voltage > 3.79) batterylevel = 2;
  else if (voltage > 3.65) batterylevel = 1;
  else batterylevel = 0;
  return batterylevel;
}

bool Power::checkForCharger(void) {
  //detection of the CC lines and adjusting charge rate
  bool result = false;
  if (analogRead(CC1) > 100 or analogRead(CC2) > 100) result = true;
  return result;
}


void Power::checkForPowerdown(void) {
  long currentTimestamp = millis();
  if (!checkForCharger())
  {
    int differenceVRxL = abs(512 - analogRead(VRxL));
    int differenceVRyL = abs(512 - analogRead(VRyL));
    if (differenceVRxL > 100 or differenceVRyL > 100)
    {
      lastActivityTime = currentTimestamp;
    }
    if ((currentTimestamp - lastActivityTime) > (PowerOffTime*60000))
    {
      disablePower();
    }
  }
  else
  {
    lastActivityTime = currentTimestamp;
  }
}

//------------------------------------------------------------------------------------------------------//
//---                                   Private methods                                              ---//
//------------------------------------------------------------------------------------------------------//

bool Power::writeI2CByte(int reg, int MSBvalue, int LSBvalue) {
  Wire1.beginTransmission(I2Cadress);
  Wire1.write(reg);
  Wire1.write(MSBvalue);
  Wire1.write(LSBvalue);
  Wire1.endTransmission();
  return true;
}

bool Power::readI2CByte(int reg, int * MSBvalue, int * LSBvalue) {
  Wire1.beginTransmission(I2Cadress);
  Wire1.write(reg);
  Wire1.endTransmission();
  Wire1.requestFrom(I2Cadress, 2);
  byte buff[2];
  Wire1.readBytes(buff, 2);
  * MSBvalue = buff[0];
  * LSBvalue = buff[1];
  return true;
}
