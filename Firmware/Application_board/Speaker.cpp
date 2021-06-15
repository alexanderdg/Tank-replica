#include<Arduino.h>
#include "Speaker.h"

#define SDCARD_CS_PIN    BUILTIN_SDCARD
#define SDCARD_MOSI_PIN  100
#define SDCARD_SCK_PIN   110




Speaker::Speaker(void) : patchCord1(playWav1, 0, audioOutput, 0), patchCord2(playWav1, 1, audioOutput, 1)
{

}

void Speaker::init(void)
{
  AudioMemory(8);
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.5);
  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  SD.begin(SDCARD_CS_PIN);
  delay(100);
  pinMode(en_vamp, OUTPUT);
  pinMode(reset_amp, OUTPUT);
  pinMode(pdn_amp, OUTPUT);
  digitalWrite(en_vamp, LOW);
  digitalWrite(reset_amp, LOW);
  digitalWrite(pdn_amp, HIGH);
  delay(1);
  digitalWrite(en_vamp, LOW);
  digitalWrite(reset_amp, HIGH);
  digitalWrite(pdn_amp, HIGH);
  delay(1);
  digitalWrite(en_vamp, HIGH);
  digitalWrite(reset_amp, HIGH);
  digitalWrite(pdn_amp, HIGH);
  delay(100);
  Wire.begin();
  Wire.setClock(100000);
  writeReg(0x1B, 0x00);
  writeReg(0x00, 0x6C);
  writeReg(0x04, 0x03);
  writeReg(0x05, 0x00);
  writeReg(0x06, 0x00);
  writeReg(0x25, 0x01, 0x00, 0x22, 0x45);
  writeReg(0x07, 0x01, 0x40);
  writeReg(0x08, 0x01, 0x40);
  writeReg(0x09, 0x01, 0x40);
}



bool Speaker::selfTest(void)
{
  return true;
}

void Speaker::playFile(const char *filename)
{
  Serial.print("Playing file: ");
  Serial.println(filename);
  playWav1.play(filename);
  //delay(1);
  /*
  while (playWav1.isPlaying()) {

  }
  */
}


void Speaker::playTank(void)
{
  playFile("TANK.WAV");
}

void Speaker::playStuck(void)
{
  playFile("STUCK.WAV");
}


void Speaker::playEmptyBattery(void)
{
  playFile("EB.WAV");
}

void Speaker::writeReg(int reg, int value1, int value2 = -1, int value3 = -1, int value4 = -1)
{
  Wire.beginTransmission(0x2A); // transmit to device #4
  Wire.write(reg);        // sends five bytes
  Wire.write(value1);              // sends one byte
  if(value2 != -1) Wire.write(value2);
  if(value3 != -1) Wire.write(value3);
  if(value4 != -1) Wire.write(value4);
  Wire.endTransmission();    // stop transmitting
}
