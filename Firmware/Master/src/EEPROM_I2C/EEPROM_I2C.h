#ifndef EEPROM_I2C_h
#define EEPROM_I2C_h

#include "Arduino.h"
#include "Wire.h"
#include "configuration/CalibrationConfig.h"

#define eepromBankAddressA 0x50
#define eepromBankAddressB 0x51

#define signatureAddress 0x00 // Check for existing calibration - char array - 2 byte - 0x00 to 0x01
// #define imuTypeAddress 0x01 // IMU type - int - 1 byte - 0x01
#define calDateAddress 0x02 // Unix date of calibration - uint32 - 4 bytes in total - 0x02 to 0x06
#define magOffsAddress 0x07 // Offsets - 3 Floats - 12 bytes in total - 0x07 to 0x13
#define magCorrAddress 0x14 // Calibration matrix - 9 Floats - 36 bytes in total - 0x14 to 0x38

#define signatureValue "OCTOSLIME" // Value for calCheckAddress

namespace EEPROM_I2C
{
    void write(int devAddr, unsigned int writeAddress, byte *data, byte len);
    void read(int devAddr, unsigned int readAddress, byte *buffer, byte len);
    void writeFloat(int devAddr, unsigned int writeAddress, float data);
    void readFloat(int devAddr, unsigned int readAddress, float *data);
    void writeCalibration(int devAddr, SlimeVR::Configuration::CalibrationConfig calibration);
    void readCalibration(int devAddr, SlimeVR::Configuration::CalibrationConfig *calibration);
    void clearCalibration(int devAddr);
    boolean checkForCalibration(int devAddr);
}

#endif