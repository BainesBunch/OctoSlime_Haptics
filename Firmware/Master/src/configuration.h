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

#ifndef SLIMEVR_CONFIG_H_
#define SLIMEVR_CONFIG_H_

namespace Octo_SlimeVR
{
    namespace Configuration
    {

        struct DeviceConfig
        {
            bool UseHaptics;
            char SSID[20];
            char Pass[20];
        };

        void getConfig();
        void setConfig(const DeviceConfig &config);
        void saveConfig();
        void initializeConfig();
        void SetWIFI(const char *ssid, const char *passphrase);
        void SetHapticsEnabled(bool Active);
        bool GetHapticsEnabled();
        void SetSSID(const char *ssid);
        void SetPassword(const char *passphrase);

    }
}
#endif // SLIMEVR_CONFIG_H_