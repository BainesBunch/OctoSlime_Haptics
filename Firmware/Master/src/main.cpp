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
#include "sensors/sensorfactory.h"
#include "configuration.h"
#include "network/network.h"
#include "globals.h"
#include "credentials.h"
#include <i2cscan.h>
#include <i2Cdev.h>
#include "serial/serialcommands.h"
#include "batterymonitor.h"
#include "UI\UI.h"
#include <MCP23017.h>

#define INT_PIN1 D6 // Make sure to connect this pin on your uC to the "INT" pin on the ICM-20948 breakout
#define INT_PIN2 D5 // Make sure to connect this pin on your uC to the "INT" pin on the ICM-20948 breakout
#define INT_RESET D7

SensorFactory sensors{};
int sensorToCalibrate = -1;
volatile bool INT_Triggered_Bank_A = false;
volatile bool INT_Triggered_Bank_B = false;
volatile uint8_t INT_Bank = A;
volatile uint8_t INT_Caller = 0;
unsigned long blinkStart = 0;
unsigned long loopTime = 0;
unsigned long last_rssi_sample = 0;
bool secondImuActive = false;
BatteryMonitor battery;

// MCP23017 myMCP = MCP23017(0x20, INT_RESET);

// IRAM_ATTR void IntBank_A(void)
// {
//     INT_Triggered_Bank_A = true;
//     INT_Bank = A;
//     INT_Caller = log(myMCP.getIntFlag(A)) / log(2);
//     //  sensors.IMU_Int_Triggered(Int_Caller);
// }

// IRAM_ATTR void IntBank_B(void)
// {
//     INT_Triggered_Bank_B = true;
//     INT_Bank = B;
//     INT_Caller = (log(myMCP.getIntFlag(B)) / log(2));
//     //   sensors.IMU_Int_Triggered(Int_Caller + 8);
// }

void setup()
{

    Serial.begin(serialBaudRate);
    SerialCommands::setUp();
    Serial.println();
    Serial.println();
    Serial.println();

    Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);

    Wire.setClockStretchLimit(150000L); // Default stretch limit 150mS
    Wire.setClock(I2C_SPEED);

    I2CSCAN::clearBus(PIN_IMU_SDA, PIN_IMU_SCL); // Make sure the bus isn't suck when reseting ESP without powering it down

    UI::Setup();
    UI::DrawSplash();
    delay(1500);
    UI::MainUIFrame();
    UI::SetMessage(1);

    // myMCP.Init();
    // myMCP.setPortMode(0b00000000, A);
    // myMCP.setPortMode(0b00000000, B);
    // myMCP.setInterruptPinPol(LOW);                 // set INTA and INTB active-high
    // myMCP.setInterruptOnChangePort(0b11111111, A); // set all B pins as interrrupt Pins
    // myMCP.setInterruptOnChangePort(0b11111111, B); // set all B pins as interrrupt Pins

    // pinMode(INT_PIN1, INPUT_PULLUP);
    // pinMode(INT_PIN2, INPUT_PULLUP);

    getConfigPtr();

    delay(500);

    sensors.create();
    sensors.init();
    sensors.motionSetup();

    Network::setUp();
    battery.Setup();
    loopTime = micros();

    // attachInterrupt(digitalPinToInterrupt(INT_PIN1), IntBank_A, FALLING); // Set up a falling interrupt
    // attachInterrupt(digitalPinToInterrupt(INT_PIN2), IntBank_B, FALLING); // Set up a falling interrupt
    // myMCP.getIntCap(B);                                                   // ensures that existing interrupts are cleared
    // myMCP.getIntCap(A);                                                   // ensures that existing interrupts are cleared
}

void loop()
{

    SerialCommands::update();
    Network::update(sensors.IMUs);
    sensors.motionLoop();
    sensors.sendData();
    battery.Loop();

    // if (INT_Triggered_Bank_A || INT_Triggered_Bank_B)
    // {
    //     switch (INT_Bank)
    //     {
    //     case A:
    //         myMCP.getIntCap(A); // ensures that existing interrupts are cleared
    //         INT_Triggered_Bank_A = false;
    //         break;
    //     case B:
    //         myMCP.getIntCap(B); // ensures that existing interrupts are cleared
    //         INT_Triggered_Bank_B = false;
    //         break;
    //     }

    //     Serial.print(F("Int Triggered : "));
    //     Serial.print(INT_Caller);

    //     Serial.print(F(" For Bank : "));
    //     Serial.println(INT_Bank);
    // }

    if (micros() - last_rssi_sample >= 2000)
    {
        last_rssi_sample = micros();
        uint8_t signalStrength = WiFi.RSSI();
        Network::sendSignalStrength(signalStrength);
    }
}
