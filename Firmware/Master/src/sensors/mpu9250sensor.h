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

#ifndef SENSORS_MPU9250SENSOR_H
#define SENSORS_MPU9250SENSOR_H

#include "sensor.h"

#include "sensors/CalibrationConfig.h"

#include <1efilter.cc>

#include <MPU9250_6Axis_MotionApps_V6_12.h>

class MPU9250Sensor : public Sensor
{
public:
    MPU9250Sensor(uint8_t address){Connected = false;addr=address;};
    ~MPU9250Sensor(){};

    boolean motionSetup() override final;
    void setupSensor(uint8_t sensorID) override final;
    void motionLoop() override final;
    void startCalibration(int calibrationType) override final;
    void getMPUScaled();
    void calibrationSetup();

private:
    MPU9250 imu{};
    bool dmpReady = false;    // set true if DMP init was successful
    uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU
    uint8_t devStatus;        // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;      // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;       // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]{}; // FIFO storage buffer
    // raw data and scaled as vector
    float q[4]{1.0f, 0.0f, 0.0f, 0.0f}; // for raw filter
    float Axyz[3]{};
    float Gxyz[3]{};
    float Mxyz[3]{};
    float rawMag[3]{};
    Quat correction{0, 0, 0, 0};
    // Loop timing globals
    unsigned long now = 0, last = 0; // micros() timers
    float deltat = 0;                // loop time in seconds

    float mag_frequency = 100.f;
    float beta = 0.02f;
    float mincutoff = 0.005f;
    float d_cutoff = 1.0f;

    OneEuroFilter f_mag_x{mag_frequency, mincutoff, beta, d_cutoff};
    OneEuroFilter f_mag_y{mag_frequency, mincutoff, beta, d_cutoff};
    OneEuroFilter f_mag_z{mag_frequency, mincutoff, beta, d_cutoff};

    Octo_SlimeVR::Configuration::MPU9250CalibrationConfig m_Calibration;

    int eepromAddr;
    
};

#endif
