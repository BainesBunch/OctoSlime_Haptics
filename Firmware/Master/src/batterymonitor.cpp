/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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
#include "batterymonitor.h"

void BatteryMonitor::Setup()
{
    loop();
}

void BatteryMonitor::Loop()
{
    auto now_ms = millis();
    if (now_ms - last_battery_sample >= batterySampleRate)
    {
        voltage = -1;

        last_battery_sample = now_ms;
        voltage = ((float)analogRead(PIN_BATTERY_LEVEL)) * batteryADCMultiplier;
        if (voltage > 0) // valid measurement
        {
            // Estimate battery level, 3.2V is 0%, 4.17V is 100% (1.0)
            if (voltage > 3.975)
                level = (voltage - 2.920) * 0.8;
            else if (voltage > 3.678)
                level = (voltage - 3.300) * 1.25;
            else if (voltage > 3.489)
                level = (voltage - 3.400) * 1.7;
            else if (voltage > 3.360)
                level = (voltage - 3.300) * 0.8;
            else
                level = (voltage - 3.200) * 0.3;

            level = (level - 0.05) / 0.95; // Cut off the last 5% (3.36V)

            if (level > 1)
                level = 1;
            else if (level < 0)
                level = 0;
            Network::sendBatteryLevel(voltage, level);
            UI::setBatStatus(level);
        }
    }
}