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
#include <i2cscan.h>
#include "sensorfactory.h"
#include "mpu6050sensor.h"
#include "ICM20948Sensor.h"
#include "mpu9250sensor.h"
//#include "bno080sensor.h"

SensorFactory::SensorFactory()
{
}

SensorFactory::~SensorFactory()
{
    for (int BankCount = 0; BankCount < 2; BankCount++)
    {
        for (int SensorCount = 0; SensorCount < IMUCount; SensorCount++)
        {
            ESP.wdtFeed();
            delete IMUs[SensorCount + (BankCount * IMUCount)];
        }
    }
}


void SensorFactory::SetIMU(uint8_t bus)
{

    Wire.beginTransmission(0x70); // TCA9548A address is 0x70
    Wire.write(1 << bus);         // send byte to select bus
    Wire.endTransmission();
}

void SensorFactory::create()
{
    Serial.println("Starting Bus Scan");

    for (int BankCount = 0; BankCount < 2; BankCount++)
    {

        for (int SensorCount = 0; SensorCount < IMUCount; SensorCount++)
        {
            this->SetIMU(SensorCount);
            I2CSCAN::DeviceParams DeviceParams = I2CSCAN::pickDevice(BankCount);
            ESP.wdtFeed();

            Serial.print("Found Device Type ID :");
            Serial.print(DeviceParams.DeviceID,HEX);
            Serial.print(" On address  :");
            Serial.print(DeviceParams.DeviceAddress,HEX);

            switch (DeviceParams.DeviceID)
            {
            case MPU6050_t:
                // Both have the same ID so magnetometer has to be checked
                Serial.println("MPU6050 detected, checking for magnetometer");
                if(this->getMagnetometerDeviceID(DeviceParams.DeviceAddress) != 0xFF){
                    this->IMUs[SensorCount + (BankCount * IMUCount)] = new MPU6050Sensor(DeviceParams.DeviceAddress);
                    this->IMUs[SensorCount + (BankCount * IMUCount)]->Connected = true;
                    Serial.println("Found MPU6050");
                } else {
                    this->IMUs[SensorCount + (BankCount * IMUCount)] = new MPU9250Sensor(DeviceParams.DeviceAddress);
                    this->IMUs[SensorCount + (BankCount * IMUCount)]->Connected = true;
                    Serial.println("Found MPU6050 + QMC5883L");
                }
                break;
             case ICM_20948_t:
                 this->IMUs[SensorCount + (BankCount * IMUCount)] = new ICM20948Sensor(DeviceParams.DeviceAddress);
                 this->IMUs[SensorCount + (BankCount * IMUCount)]->Connected = true;
                 Serial.println("Found ICM20948");
                 break;

            // case BNO_080_t:
            //     this->IMUs[SensorCount + (BankCount * IMUCount)] = new BNO080Sensor(DeviceParams.DeviceAddress);
            //     this->IMUs[SensorCount + (BankCount * IMUCount)]->Connected = true;
            //     Serial.println("Found BNO080");
            //     break;

            default:
                // if (DeviceParams.DeviceAddress == 0x4A || DeviceParams.DeviceAddress == 0x4B) // fallback for the BNO IMU
                // {
                //     // RPB this->IMUs[SensorCount + (BankCount * IMUCount)] = new BNO080Sensor(DeviceParams.DeviceAddress);
                //     this->IMUs[SensorCount + (BankCount * IMUCount)]->Connected = true;
                //     Serial.println("Found BNO080");
                // }
                // else
                // {

                    this->IMUs[SensorCount + (BankCount * IMUCount)] = new EmptySensor();
                    this->IMUs[SensorCount + (BankCount * IMUCount)]->Connected = false;
                    Serial.println("Nothing Found");
                // }
                 break;
            }
        }
    }
}

void SensorFactory::init()
{
    Serial.println("Setting up IMU Parameters");

    for (int BankCount = 0; BankCount < 2; BankCount++)
    {

        for (int SensorCount = 0; SensorCount < IMUCount; SensorCount++)
        {
            ESP.wdtFeed();

            uint8_t IMUID = SensorCount + (BankCount * IMUCount);
            SetIMU(SensorCount);
            Serial.print("Setting IMU ID : ");
            Serial.print(IMUID);
            if (IMUs[IMUID]->Connected)
            {
                IMUs[IMUID]->setupSensor(SensorCount + (BankCount * IMUCount));
                Serial.println(" Complete");
            }
            else
            {
                Serial.println(" No Device");
            }
        }
    }
}

void SensorFactory::motionSetup()
{
    Serial.println("Setting up Motion Engines");

    for (int BankCount = 0; BankCount < 2; BankCount++)
    {
        for (int SensorCount = 0; SensorCount < IMUCount; SensorCount++)
        {
            ESP.wdtFeed();

            uint8_t IMUID = SensorCount + (BankCount * IMUCount);
            SetIMU(SensorCount);
            Serial.print("Setting IMU ID : ");
            Serial.print(IMUID);

            if (IMUs[IMUID]->Connected)
            {
                ESP.wdtDisable();
                IMUs[IMUID]->motionSetup();
                ESP.wdtEnable(WDTO_500MS);
                UI::SetIMUStatus(IMUID, IMUs[IMUID]->isWorking() ? true : false);
                Serial.println(" Complete");
            }
            else
            {
                Serial.println(" No Device");
                UI::SetIMUStatus(IMUID, false);
            }
        }
    }
}

void SensorFactory::motionLoop()
{
    for (int BankCount = 0; BankCount < 2; BankCount++)
    {
        for (int SensorCount = 0; SensorCount < IMUCount; SensorCount++)
        {
            ESP.wdtFeed();
            this->SetIMU(SensorCount);
            uint8_t IMUID = SensorCount + (BankCount * IMUCount);
            if (IMUs[IMUID]->Connected && IMUs[IMUID]->isWorking())
            {
                if (IMUs[IMUID]->getSensorState() == SENSOR_OK)
                {
                    IMUs[IMUID]->motionLoop();
                }
                else
                {
                    Serial.printf("Sensor ID %d Offline", IMUID);
                }
            }
        }
    }
}

void SensorFactory::sendData()
{
    for (int BankCount = 0; BankCount < 2; BankCount++)
    {
        for (int SensorCount = 0; SensorCount < IMUCount; SensorCount++)
        {
            uint8_t IMUID = SensorCount + (BankCount * IMUCount);
            if (IMUs[IMUID]->Connected && IMUs[IMUID]->isWorking())
            {
                if (IMUs[IMUID]->getSensorState() == SENSOR_OK && IMUs[IMUID]->newData)
                {
                    ESP.wdtFeed();
                    this->SetIMU(SensorCount);
                    IMUs[IMUID]->sendData();
                }
            }
        }
    }
}

void SensorFactory::startCalibration(int sensorId, int calibrationType)
{
    this->SetIMU(sensorId);
    if (IMUs[sensorId]->Connected && IMUs[sensorId]->isWorking())
    {
        ESP.wdtFeed();
        IMUs[sensorId]->startCalibration(calibrationType);
    }
}

uint8_t SensorFactory::getMagnetometerDeviceID(uint8_t addr) //check lib\mpu9250\MPU9250.cpp for reference
{
    uint8_t buffer[14];

    // Set master mode to true
    I2Cdev::writeBit(addr, 0x6A, 5, true);
    delay(50);

    // Set up magnetometer as slave 0 for reading
    I2Cdev::writeByte(addr, 0x25, 0x0D | 0x80);
    delay(10);
    // Start reading from WHO_AM_I register
    I2Cdev::writeByte(addr, 0x26, 0x0D);
    I2Cdev::writeByte(addr, 0x27, 0x81);
    delay(10);
    I2Cdev::readByte(addr, 0x49, buffer);
    // return reading from HXL register
    I2Cdev::writeByte(addr, 0x26, 0x00);
    delay(10);
    // Read 7 bytes (until ST2 register), group LSB and MSB
    I2Cdev::writeByte(addr, 0x26, 0x96);
    delay(50);

    // Set master mode to false
    I2Cdev::writeBit(addr, 0x6A, 5, false);

    return buffer[0];
}
