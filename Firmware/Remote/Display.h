#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include "SPI.h"
#include "ILI9341_t3.h"
#include "font_Arial.h"
#include "Power.h"
#include "Xbee.h"

#define TFT_DC 9
#define TFT_CS 10
#define TFT_RST 8
#define TFT_MOSI 11;
#define TFT_CLK 13;
#define TFT_MISO 12;
#define TFT_BL_EN 7


class Display {
  public:
    Display();
    void initDisplay(void);
    void enableDisplay(void);
    void disableDisplay(void);
    
    bool testDisplay(void);
    bool drawStartScreen(void);
    bool updateInfo(Power * power, Xbee * xbee, int driveMode);

    void drawChargeSymbol(bool enable);
    void drawBatterySymbol(int level);
    void drawTankVoltage(float voltage);
    void drawRemoteVoltage(float voltage);
    void drawRemoteCurrent(float current);
    void drawDriveMode(int mode);
    void drawLeftMotorCurrent(float current);
    void drawRightMotorCurrent(float current);
    void drawLeftMotorSpeed(float speed);
    void drawRightMotorSpeed(float speed);
    void drawNoConnection(bool connection);

  private:
    ILI9341_t3 tft;

    bool noConnectionBoxDrawn = true;
    bool drawChargeSymbolDrawn = false;
    int batterySymbolDrawn = -1;
    float tankVoltageDrawn = -2.0;
    float remoteVoltageDrawn = -2.0;
    float remoteCurrentDrawn = -2.0;
    int driveModeDrawn = 0;
    float leftMotorCurrentDrawn = -2.0;
    float rightMotorCurrentDrawn = -2.0;
    float leftMotorSpeedDrawn = -2.0;
    float rightMotorSpeedDrawn = -2.0;
    long batterySymbolTimeDrawn = -10000;
};

#endif
