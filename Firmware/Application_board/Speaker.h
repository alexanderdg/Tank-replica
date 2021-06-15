#ifndef SPEAKER_H
#define SPEAKER_H

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <Arduino.h>

class Speaker
{
  public:
    Speaker();
    void init(void);
    void playFile(const char *filename);
    void playTank(void);
    void playStuck(void);
    void playEmptyBattery(void);

    bool selfTest(void);

  private:
    void writeReg(int reg, int value1, int value2 = -1, int value3 = -1, int value4 = -1);

    int en_vamp = 8;
    int reset_amp = 3;
    int pdn_amp = 2;

    AudioPlaySdWav playWav1;
    AudioOutputI2S audioOutput;
    AudioConnection patchCord1;
    AudioConnection patchCord2;
    AudioControlSGTL5000 sgtl5000_1;

};





#endif
