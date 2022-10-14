/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "Wire.h"
#include "ota.h"
#include "sensors/sensorfactory.h"
#include "configuration.h"
#include "network/network.h"
#include "globals.h"
#include "credentials.h"
#include <i2cscan.h>
#include "serial/serialcommands.h"
#include "batterymonitor.h"
#include "UI\UI.h"
#include <INT_Marshal/DFRobot_MCP23017.h>
#include "LEDManager.h"
#include "status/StatusManager.h"

SlimeVR::Configuration::Configuration configuration; // thanks Emojikage#3095 for telling me this
SlimeVR::LEDManager ledManager(LED_PIN);
SlimeVR::Status::StatusManager statusManager;

SensorFactory sensors{};
int sensorToCalibrate = -1;
bool blinking = false;
unsigned long blinkStart = 0;
unsigned long loopTime = 0;
unsigned long last_rssi_sample = 0;
bool secondImuActive = false;
BatteryMonitor battery;

DFRobot_MCP23017 INT_Marshal;

bool intFlagA = false; // INTA interrupt sign
bool intFlagB = false; // INTB interrupt sign

char *int2bin(uint8_t x)
{
  static char buffer[9];
  for (int i = 0; i < 8; i++)
    buffer[7 - i] = '0' + ((x & (1 << i)) > 0);
  buffer[8] = '\0';
  return buffer;
}

void IMU_Int_Service_A(uint8_t Index)
{
  String description = INT_Marshal.pinDescription(Index);
  Serial.print(description);
  Serial.println("Port A Interruption occurs!");
}

void IMU_Int_Service_B(uint8_t Index)
{
  String description = INT_Marshal.pinDescription(Index);
  Serial.print(description);
  Serial.println("Port B Interruption occurs!");
  sensors.INT_Triggered(Index);
}

IRAM_ATTR void notifyA()
{
  intFlagA = true;
}
IRAM_ATTR void notifyB()
{
  intFlagB = true;
}

void setup()
{

  pinMode(INT_PIN_1, INPUT_PULLUP);
  pinMode(INT_PIN_2, INPUT_PULLUP);

  INT_Marshal.begin();

  INT_Marshal.pinModeInterrupt(INT_Marshal.eGPA, INT_Marshal.eChangeLevel, IMU_Int_Service_A);
  INT_Marshal.pinModeInterrupt(INT_Marshal.eGPB, INT_Marshal.eChangeLevel, IMU_Int_Service_B);

  attachInterrupt(INT_PIN_1, notifyA, RISING);
  attachInterrupt(INT_PIN_2, notifyB, RISING);

  statusManager.setStatus(SlimeVR::Status::LOADING, true);
  ledManager.setup();
  configuration.setup();

  // UI::Setup();
  // UI::DrawSplash();
  // delay(1500);
  // UI::MainUIFrame();
  // UI::SetMessage(1);

  Serial.begin(serialBaudRate);
  SerialCommands::setUp();
  Serial.println();
  Serial.println();
  Serial.println();
  I2CSCAN::clearBus(PIN_IMU_SDA, PIN_IMU_SCL); // Make sure the bus isn't suck when reseting ESP without powering it down
  Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);
  Wire.setClockStretchLimit(150000L); // Default stretch limit 150mS
  Wire.setClock(I2C_SPEED);

  getConfigPtr();

  delay(500);

  sensors.create();
  sensors.motionSetup();

  Network::setUp();
  OTA::otaSetup(otaPassword);
  battery.Setup();
  loopTime = micros();

  statusManager.setStatus(SlimeVR::Status::LOADING, false);

}

void loop()
{

  if (intFlagA)
  {
    intFlagA = false;
    INT_Marshal.pollInterrupts(INT_Marshal.eGPIOA);
  }

  if (intFlagB)
  {
    intFlagB = false;
    INT_Marshal.pollInterrupts(INT_Marshal.eGPIOB);
  }

  SerialCommands::update();
  OTA::otaUpdate();
  Network::update(sensors.IMUs);
  sensors.motionLoop(); // culprit right now
  sensors.sendData();
  battery.Loop();
  ledManager.update();
  if (millis() - last_rssi_sample >= 2000)
  {
    last_rssi_sample = millis();
    uint8_t signalStrength = WiFi.RSSI();
    Network::sendSignalStrength(signalStrength);
  }
}
