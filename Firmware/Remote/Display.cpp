#include "Display.h"
#include <Arduino.h>
#include "SPI.h"
#include "ILI9341_t3.h"
#include "font_Arial.h"
#include <font_AwesomeF080.h>
#include <font_AwesomeF200.h>

Display::Display(void) : tft(TFT_CS, TFT_DC, 8, 11, 13, 12)
{
  pinMode(TFT_BL_EN, OUTPUT);
}

void Display::initDisplay(void) {
  digitalWrite(TFT_BL_EN, HIGH);
  tft.begin();
  tft.setClock(60000000);
}

void Display::enableDisplay(void) {
  digitalWrite(TFT_RST, HIGH);
  delay(500);
  digitalWrite(TFT_BL_EN, HIGH);
  tft.begin();
  tft.sleep(false);
  tft.setClock(60000000);
  tft.setRotation(1);
  tft.fillRect(0, 0, 320, 240, ILI9341_WHITE);
}

void Display::disableDisplay(void) {
  digitalWrite(TFT_BL_EN, LOW);
  digitalWrite(TFT_RST, LOW);
  tft.sleep(true);
  noConnectionBoxDrawn = true;
  drawChargeSymbolDrawn = false;
  batterySymbolDrawn = -1;
  tankVoltageDrawn = -2.0;
  remoteVoltageDrawn = -2.0;
  remoteCurrentDrawn = -2.0;
  driveModeDrawn = 0;
  leftMotorCurrentDrawn = -2.0;
  rightMotorCurrentDrawn = -2.0;
  leftMotorSpeedDrawn = -2.0;
  rightMotorSpeedDrawn = -2.0;
}

bool Display::drawStartScreen(void) {
  tft.fillScreen(ILI9341_WHITE);
  tft.setRotation(1);
  tft.fillRect(0, 30, 320, 2, ILI9341_BLACK);
  tft.fillRect(0, 170, 320, 2, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_BLACK);
  tft.setFont(Arial_14);
  tft.setCursor(20, 180);
  tft.print("Left motor:");
  tft.setCursor(190, 180);
  tft.print("Right motor:");
}

bool Display::updateInfo(Power * power, Xbee * xbee, int driveMode) {
  //tft.fillRect(0, 5, 320, 23, ILI9341_WHITE);
  drawChargeSymbol(power -> checkForCharger());
  drawBatterySymbol(power -> readCapacity());
  drawTankVoltage(xbee -> getTankVoltage());
  drawRemoteVoltage(power -> readVoltage());
  drawRemoteCurrent(power -> readCurrent());
  drawDriveMode(driveMode);
  drawLeftMotorCurrent(xbee -> getTankCurrentLeftMotor());
  drawRightMotorCurrent(xbee -> getTankCurrentRightMotor());
  drawLeftMotorSpeed(xbee -> getTankSpeedLeftMotorMS());
  drawRightMotorSpeed(xbee -> getTankSpeedRightMotorMS());
  drawNoConnection(xbee -> checkForConnection());
}

bool Display::testDisplay(void) {
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_WHITE);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(3);
  tft.setCursor(10, 10);
  tft.println("Tank remote....");
  delay(1000);
  tft.setRotation(2);
  tft.fillScreen(ILI9341_WHITE);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(3);
  tft.setCursor(10, 10);
  tft.println("Tank remote....");
  delay(1000);
  tft.setRotation(3);
  tft.fillScreen(ILI9341_WHITE);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(3);
  tft.setCursor(10, 10);
  tft.println("Tank remote....");
  return true;
}

void Display::drawChargeSymbol(bool enable) {
  if (enable && !drawChargeSymbolDrawn) {
    drawChargeSymbolDrawn = true;
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_BLACK);
    tft.setFont(AwesomeF080_18);
    tft.setCursor(5, 3);
    tft.print((char)103);
  }
  else if (!enable && drawChargeSymbolDrawn) {
    drawChargeSymbolDrawn = false;
    tft.fillRect(0, 5, 30, 23, ILI9341_WHITE);
  }
}

void Display::drawBatterySymbol(int level) {
  long currentTimestamp = millis();
  if (level != batterySymbolDrawn && (currentTimestamp - batterySymbolTimeDrawn) > 10000)
  {
    batterySymbolTimeDrawn = currentTimestamp;
    batterySymbolDrawn = level;
    tft.fillRect(35, 5, 40, 23, ILI9341_WHITE);
    tft.setTextSize(2);
    switch (level) {
      case 0:
        tft.setTextColor(ILI9341_RED);
        break;
      case 1:
        tft.setTextColor(ILI9341_RED);
        break;
      case 2:
        tft.setTextColor(ILI9341_ORANGE);
        break;
      case 3:
        tft.setTextColor(ILI9341_DARKGREEN);
        break;
      case 4:
        tft.setTextColor(ILI9341_GREEN);
        break;
    }
    tft.setFont(AwesomeF200_18);
    tft.setCursor(40, 3);
    tft.print((char)(68 - level));
    tft.setCursor(40, 3);
    if (level == 0) tft.setTextColor(ILI9341_RED);
    else tft.setTextColor(ILI9341_BLACK);
    tft.print((char)(68));
  }
}

void Display::drawTankVoltage(float voltage) {
  if (voltage != tankVoltageDrawn)
  {
    tankVoltageDrawn = voltage;
    tft.fillRect(83, 6, 67, 17, ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_BLACK);
    tft.setFont(Arial_14);
    tft.setCursor(90, 7);
    tft.print(voltage);
    tft.print("V ");
  }
}


void Display::drawRemoteVoltage(float voltage) {
  if (voltage != remoteVoltageDrawn)
  {
    remoteVoltageDrawn = voltage;
    tft.fillRect(153, 6, 67, 17, ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_BLACK);
    tft.setFont(Arial_14);
    tft.setCursor(160, 7);
    tft.print(voltage);
    tft.print("V ");
  }
}


void Display::drawRemoteCurrent(float current) {
  if (current != remoteCurrentDrawn)
  {
    remoteCurrentDrawn = current;
    tft.fillRect(213, 6, 67, 17, ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_BLACK);
    tft.setFont(Arial_14);
    tft.setCursor(220, 7);
    tft.print(current);
    tft.print("A");
  }
}

void Display::drawDriveMode(int mode) {
  if (mode != driveModeDrawn)
  {
    driveModeDrawn = mode;
    tft.fillRect(288, 2, 30, 26, ILI9341_WHITE);
    tft.drawRect(290, 3, 24, 24, ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_BLACK);
    tft.setFont(Arial_14);
    tft.setCursor(295, 8);
    switch (mode)
    {
      case 1:
        tft.fillRect(291, 4, 22, 22, ILI9341_GREEN);
        tft.print("S");
        break;
      case 2:
        tft.fillRect(291, 4, 22, 22, ILI9341_ORANGE);
        tft.print("M");
        break;
      case 3:
        tft.fillRect(291, 4, 22, 22, ILI9341_RED);
        tft.print("F");
        break;
    }
  }
}


void Display::drawLeftMotorCurrent(float current) {
  if (current != leftMotorCurrentDrawn)
  {
    leftMotorCurrentDrawn = current;
    tft.fillRect(50, 200, 70, 17, ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_BLACK);
    tft.setFont(Arial_14);
    tft.setCursor(20, 200);
    tft.print("Im: ");
    tft.print(current);
    tft.print("A");
  }
}


void Display::drawRightMotorCurrent(float current) {
  if (current != rightMotorCurrentDrawn)
  {
    rightMotorCurrentDrawn = current;
    tft.fillRect(220, 200, 70, 17, ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_BLACK);
    tft.setFont(Arial_14);
    tft.setCursor(190, 200);
    tft.print("Im: ");
    tft.print(current);
    tft.print("A");
  }
}


void Display::drawLeftMotorSpeed(float speed) {
  if (speed != leftMotorSpeedDrawn)
  {
    leftMotorSpeedDrawn = speed;
    tft.fillRect(50, 220, 80, 17, ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_BLACK);
    tft.setFont(Arial_14);
    tft.setCursor(20, 220);
    tft.print("V=: ");
    tft.print(speed);
    tft.print("m/s");
  }
}


void Display::drawRightMotorSpeed(float speed) {
  if (speed != rightMotorSpeedDrawn)
  {
    rightMotorSpeedDrawn = speed;
    tft.fillRect(220, 220, 80, 17, ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_BLACK);
    tft.setFont(Arial_14);
    tft.setCursor(190, 220);
    tft.print("V=: ");
    tft.print(speed);
    tft.print("m/s");
  }
}


void Display::drawNoConnection(bool connection) {
  if (!connection && noConnectionBoxDrawn == true)
  {
    tft.fillRect(10, 70, 300, 60, ILI9341_RED);
    tft.setTextSize(4);
    tft.setTextColor(ILI9341_WHITE);
    tft.setFont(Arial_20);
    tft.setCursor(60, 88);
    tft.print("No Connection!");
    noConnectionBoxDrawn = false;
  }
  else if (connection && noConnectionBoxDrawn == false)
  {
    noConnectionBoxDrawn = true;
    tft.fillRect(0, 70, 320, 60, ILI9341_WHITE);
  }
}
