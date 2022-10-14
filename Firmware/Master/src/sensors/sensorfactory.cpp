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
#include "sensorfactory.h"
#include "ICM20948Sensor.h"
#include "bno080sensor.h"
#include "mpu6050sensor.h"
#include "mpu9250sensor.h"
#include "sensor.h"
#include <i2cscan.h>
#include "EEPROM_I2C/EEPROM_I2C.h"

boolean Sensor_Calibrated = false;

SensorFactory::SensorFactory()
{

}

void SensorFactory::IMU_Int_Triggered(uint8_t IMU_ID)
{
    Serial.print(F("Int Triggerd : "));
    Serial.println(IMU_ID);
    if (this->IMUs[IMU_ID]->getSensorType() == BNO_080_t)
    {
        this->IMUs[IMU_ID]->Int_Fired();
    }
}

boolean SensorFactory::GetSensorOnline(uint8_t IMU_ID)
{
    return (this->IMUs[IMU_ID]->getSensorState() == SENSOR_OK);
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
            Serial.print(DeviceParams.DeviceID, HEX);
            Serial.print(" On address  :");
            Serial.print(DeviceParams.DeviceAddress, HEX);

            switch (DeviceParams.DeviceID)
            {
            case MPU6050_t:
                // Both have the same ID so magnetometer has to be checked
                Serial.println("MPU6050 detected, checking for magnetometer");
                if (this->getMagnetometerDeviceID(DeviceParams.DeviceAddress) != 0xFF)
                {
                    this->IMUs[SensorCount + (BankCount * IMUCount)] = new MPU6050Sensor(DeviceParams.DeviceAddress);
                    this->IMUs[SensorCount + (BankCount * IMUCount)]->Connected = true;
                    Serial.println("Found MPU6050");
                }
                else
                {
                    this->IMUs[SensorCount + (BankCount * IMUCount)] = new MPU9250Sensor(DeviceParams.DeviceAddress);
                    this->IMUs[SensorCount + (BankCount * IMUCount)]->Connected = true;
                    Serial.println("Found MPU6050 + QMC5883L");

                    // if (configuration.getCalibration(SensorCount + (BankCount + IMUCount)).type != SlimeVR::Configuration::CalibrationConfigType::MPU9250) {
                    //     UI::DrawCalibrationAdvice(SensorCount + (BankCount + IMUCount));
                    // }

                    // if(EEPROM_I2C::checkForCalibration((SensorCount + (BankCount * IMUCount)) < IMUCount ? eepromBankAddressA : eepromBankAddressB)){
                    // UI::DrawCalibrationAdvice(SensorCount + (BankCount * IMUCount));
                    // }
                }
                break;
            case ICM_20948_t:
                this->IMUs[SensorCount + (BankCount * IMUCount)] = new ICM20948Sensor(DeviceParams.DeviceAddress);
                this->IMUs[SensorCount + (BankCount * IMUCount)]->Connected = true;
                Serial.println("Found ICM20948");
                break;

            default:
                if (DeviceParams.DeviceAddress == 0x4A || DeviceParams.DeviceAddress == 0x4B) // fallback for the BNO IMU
                {
                    this->IMUs[SensorCount + (BankCount * IMUCount)] = new BNO080Sensor(DeviceParams.DeviceAddress);
                    this->IMUs[SensorCount + (BankCount * IMUCount)]->Connected = true;
                    Serial.println("Found BNO080");
                }
                else
                {

                    this->IMUs[SensorCount + (BankCount * IMUCount)] = new EmptySensor();
                    this->IMUs[SensorCount + (BankCount * IMUCount)]->Connected = false;
                    Serial.println("Nothing Found");
                }
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

boolean SensorFactory::CalibrationEvent()
{
    return Sensor_Calibrated;
}

void SensorFactory::clearCalibrations()
{
    for (int BankCount = 0; BankCount < 2; BankCount++)
    {
        for (int SensorCount = 0; SensorCount < IMUCount; SensorCount++)
        {
            SetIMU(SensorCount + (BankCount * IMUCount));
            EEPROM_I2C::clearCalibration((SensorCount + (BankCount * IMUCount)) < IMUCount ? eepromBankAddressA : eepromBankAddressB);
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
                Sensor_Calibrated |= IMUs[IMUID]->motionSetup();
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

uint8_t SensorFactory::getMagnetometerDeviceID(uint8_t addr) // check lib\mpu9250\MPU9250.cpp for reference
{
    uint8_t buffer[14];

    // set up MPU 6050
    I2Cdev::writeBits(addr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CLKSEL_BIT, MPU9250_PWR1_CLKSEL_LENGTH, MPU9250_CLOCK_PLL_XGYRO);
    I2Cdev::writeBit(addr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_SLEEP_BIT, false);

    delay(100);

    // disable master mode
    I2Cdev::writeBit(addr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_I2C_MST_EN_BIT, false);
    delay(100);
    // enable bypass mode
    I2Cdev::writeBit(addr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_I2C_BYPASS_EN_BIT, true);
    delay(100);
    // read nothing
    I2Cdev::readByte(0x0D, 0x00, buffer);
    delay(10);
    // read whoami
    I2Cdev::readByte(0x0D, 0x0D, buffer);
    delay(100);
    // disable bypass mode
    I2Cdev::writeBit(addr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_I2C_BYPASS_EN_BIT, false);
    delay(100);
    return buffer[0];
}
