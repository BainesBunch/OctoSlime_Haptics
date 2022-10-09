#include "Arduino.h"
#include "Wire.h"
#include "EEPROM_I2C.h"

namespace EEPROM_I2C
{

    void write(int devAddr, unsigned int writeAddress, byte *data, byte len)
    {
        Wire.beginTransmission(devAddr);
        Wire.write((int)(writeAddress >> 8));
        Wire.write((int)(writeAddress & 0xFF));
        for (byte c = 0; c < len; c++)
        {
            Wire.write(data[c]);
        }
        Wire.endTransmission();
    }

    void read(int devAddr, unsigned int readAddress, byte *buffer, byte len)
    {
        Wire.beginTransmission(devAddr);
        Wire.write((int)(readAddress >> 8));
        Wire.write((int)(readAddress & 0xFF));
        Wire.endTransmission();
        Wire.requestFrom(devAddr, len);
        for (byte c = 0; c < len; c++)
        {
            if (Wire.available())
            {
                buffer[c] = Wire.read();
            }
        }
    }

    void writeFloat(int devAddr, unsigned int writeAddress, float data)
    {
        write(devAddr, writeAddress, (byte *)&data, 4);
    }

    void readFloat(int devAddr, unsigned int readAddress, float *data)
    {
        byte buffer[4];
        read(devAddr, readAddress, buffer, 4);
        *data = *(float *)buffer;
    }

    void writeCalibration(int devAddr, SlimeVR::Configuration::CalibrationConfig calibration)
    {
        write(devAddr, signatureAddress, (byte *) signatureValue, 2);

        // TODO: Add function for time and date in form of uint32
        //  leaving calibration date for now
        

        switch (calibration.type)
        {
        case SlimeVR::Configuration::CalibrationConfigType::MPU9250:

            // Offset write
            for (int i = 0; i < 3; i++)
            {
                // Float has 4 bytes so address needs to increase by 4 for each float
                writeFloat(devAddr, magOffsAddress + (i * 4), calibration.data.mpu9250.M_B[i]);
            }

            // Calibration matrix write
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    // Float has 4 bytes so address needs to increase by 4 for each float
                    writeFloat(devAddr, magOffsAddress + (i * j * 4), calibration.data.mpu9250.M_Ainv[i][j]);
                }
            }
            break;
        default:
            break;
        }
    }

    void readCalibration(int devAddr, SlimeVR::Configuration::CalibrationConfig *calibration)
    {
        // TODO: Add function for time and date in form of uint32
        //  leaving calibration date for now

        switch (calibration->type)
        {
        case SlimeVR::Configuration::CalibrationConfigType::MPU9250:
            float read;

            // Offset read
            for (int i = 0; i < 3; i++)
            {
                readFloat(devAddr, magOffsAddress + (i * 4), &read);
                calibration->data.mpu9250.M_B[i] = read;
            }

            // Calibration matrix read
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    readFloat(devAddr, magCorrAddress + (i * j * 4), &read);
                    calibration->data.mpu9250.M_Ainv[i][j] = read;
                }
            }

            break;
        default:
            calibration->type = SlimeVR::Configuration::CalibrationConfigType::NONE;
            break;
        }
    }

    void clearCalibration(int devAddr)
    {
        write(devAddr, 0x00, 0, 0x38);
    }

    boolean checkForCalibration(int devAddr)
    {
        byte buff[2];
        read(devAddr, signatureAddress, buff, 2);
        return ((char*)buff) == signatureValue;
    }

}
