#ifndef EEPROM_I2C_h
#define EEPROM_I2C_h

#include "Arduino.h"
#include "Wire.h"
#include "sensors/CalibrationConfig.h"

#define eepromBankAddressA 0x50
#define eepromBankAddressB 0x51

#define signatureAddress 0x00 // Check for existing calibration - char array - 10 byte - 0x00 to 0x09
// #define imuTypeAddress 0x01 // IMU type - int - 1 byte - 0x01
#define calDateAddress 0x0A // Unix date of calibration - uint32 - 4 bytes in total - 0x0A to 0x0D
#define magOffsAddress 0x0E // Offsets - 3 Floats - 12 bytes in total - 0x0E to 0x19
#define magCorrAddress 0x1A // Calibration matrix - 9 Floats - 36 bytes in total - 0x1A to 0x3E

#define signatureValue "OCTOSLIME" // Value for calCheckAddress

namespace EEPROM_I2C {
void write(int devAddr, unsigned int writeAddress, byte* data, int len);
void writeWithOffset(int devAddr, unsigned int writeAddress, byte* data, int offset, int len);
void read(int devAddr, unsigned int readAddress, byte* buffer, int len);
void readWithOffset(int devAddr, unsigned int readAddress, byte* buffer, int offset, int len);
void writeByte(int devAddr, unsigned int writeAddress, byte data);
byte readByte(int devAddr, unsigned int readaddress);
int writeFloat(int devAddr, unsigned int writeAddress, float data);
float readFloat(int devAddr, unsigned int readAddress);
void writeCalibration(int devAddr, Octo_SlimeVR::Configuration::CalibrationConfig calibration);
void readCalibration(int devAddr, Octo_SlimeVR::Configuration::CalibrationConfig* calibration);
void clearCalibration(int devAddr);
unsigned long getDate(int devAddr);
int checkForCalibration(int devAddr);
void test();
void ack_pooling(int devAddr);
}

#endif