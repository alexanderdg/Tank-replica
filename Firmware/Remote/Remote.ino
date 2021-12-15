#include <Arduino.h>
#include "Display.h"
#include "Power.h"
#include "Xbee.h"
#include <Snooze.h>
#include <LoRaLib.h>

#define LORA_CS 31
#define LORA_DIO0 3
#define LORA_DIO1 4
#define VRxL A3
#define VRyL A2


IntervalTimer tickTimer;
Display display;
Power power;
//SX1278 lora = new LoRa(LORA_CS, LORA_DIO0, LORA_DIO1, SPI1);

SnoozeTimer timer;
SnoozeBlock config(timer);
Xbee xbee;
long savedTimestamp = 0;
float tankVoltage = -1.00;
bool deepsleep = false;
int driveMode = 3;

void setup() {
  // put your setup code here, to run once
  Serial5.begin(115200);
  timer.setTimer(5000);
  power.initPower();
  display.initDisplay();
  display.drawStartScreen();
  Serial.begin(115200);
  power.setChargeRate1A5(true);
  tickTimer.begin(letItTick, 20000);
  xbee.initXbee();
}

void loop() {
  if (power.readVoltage() < 3.3) {
    display.disableDisplay();
    xbee.disableXbee();
    deepsleep = true;
    int who = Snooze.deepSleep( config );
  }
  else {
    if (deepsleep ) {
      if (power.readVoltage() > 3.5)
      {
        delay(1000);
        deepsleep = false;
        xbee.enableXbee();
        display.enableDisplay();
        display.drawStartScreen();
      }
      else {
        Snooze.deepSleep( config );
      }
    }
    else if ((millis() - savedTimestamp) > 250)
    {
      display.updateInfo(&power, &xbee, driveMode);
      power.checkForPowerdown();
      savedTimestamp = millis();
    }
  }

}

void serialEvent5() {
  xbee.readXbeeData();
}

void letItTick(void) {
  char message[6];
  message[0] = analogRead(VRxL) >> 2;
  message[1] = analogRead(VRyL) >> 2;
  message[2] = 0;
  message[3] = 0;
  message[4] = driveMode;
  message[5] = ':';
  for (int i = 0; i < 6; i++) {
    Serial5.print(message[i]);
  }
}

void startLoRaModule(void) {
  /*
    digitalWrite(LORA_RST, HIGH);
    delay(500);
    Serial.print(F("Initializing SX1278 ... "));
    int state = lora.beginFSK(434.0, 200, 100, 250, 17, 100, 16, false);
    lora.setNodeAddress(0x01);
    lora.setDio0Action(setFlag);
    lora.startReceive();
    if (state == ERR_NONE) {
    Serial.println(F("success!"));
    } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    }
  */
}

void setFlag(void) {
  // check if the interrupt is enabled
  /*
    if (!enableInterrupt) {
    return;
    }
    receiveFlag = true;
  */
}
