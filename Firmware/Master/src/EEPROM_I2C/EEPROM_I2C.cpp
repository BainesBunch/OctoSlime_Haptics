#include "EEPROM_I2C.h"
#include "Arduino.h"
#include "Wire.h"
#include "network/network.h"

namespace EEPROM_I2C {

byte _pageSize = 64;

void ack_pooling(int devAddr)
{
    byte code = -1;
    do {
        Wire.beginTransmission(devAddr);
        Wire.write((byte)0xFF);
        code = Wire.endTransmission();
        Serial.printf("ack_pooling: %d\n", code);
        ESP.wdtFeed();
    } while (code != 0);
}

void writeByte(int devAddr, unsigned int address, byte data)
{
    Wire.beginTransmission(devAddr);
    Wire.write(0);
    if (Wire.endTransmission() == 0) {
        Wire.beginTransmission(devAddr);
        Wire.write(address >> 8);
        Wire.write(address & 0xFF);
        Wire.write(data);
        Wire.endTransmission();
        delay(20);
    } else {
        Serial.println("endTransmission exited with error code");
    }
}

byte readByte(int devAddr, unsigned int readAddress)
{
    ack_pooling(devAddr);
    Wire.beginTransmission(devAddr);
    Wire.write((byte)(readAddress >> 8));
    Wire.write((byte)(readAddress & 0xFF));
    Wire.endTransmission();
    Wire.requestFrom(devAddr, 1);
    Wire.available();
    return Wire.read();
}

void write(int devAddr, unsigned int writeAddress, byte* data, int n)
{
    // status quo
    int c = n; // bytes left to write
    int offD = 0; // current offset in data pointer
    int offP; // current offset in page
    int nc = 0; // next n bytes to write

    // write alle bytes in multiple steps
    while (c > 0) {
        // calc offset in page
        offP = writeAddress % _pageSize;
        // maximal 30 bytes to write
        nc = min(min(c, 30), _pageSize - offP);
        writeWithOffset(devAddr, writeAddress, data, offD, nc);
        c -= nc;
        offD += nc;
        writeAddress += nc;
    }
}

void writeWithOffset(int devAddr, unsigned int writeAddress, byte* data, int offset, int n)
{
    Wire.beginTransmission(devAddr);
    if (Wire.endTransmission() == 0) {
        Wire.beginTransmission(devAddr);
        Wire.write(writeAddress >> 8);
        Wire.write(writeAddress & 0xFF);
        byte* adr = data + offset;
        Wire.write(adr, n);
        Wire.endTransmission();
        delay(20);
    }
}

void read(int devAddr, unsigned int readAddress, byte* data, int n)
{
    int c = n;
    int offD = 0;
    // read until are n bytes read
    while (c > 0) {
        // read maximal 32 bytes
        int nc = c;
        if (nc > 32)
            nc = 32;
        readWithOffset(devAddr, readAddress, data, offD, nc);
        readAddress += nc;
        offD += nc;
        c -= nc;
    }
}

void readWithOffset(int devAddr, unsigned int readAddress, byte* data, int offset, int n)
{
    Wire.beginTransmission(devAddr);
    if (Wire.endTransmission() == 0) {
        Wire.beginTransmission(devAddr);
        Wire.write(readAddress >> 8);
        Wire.write(readAddress & 0xFF);
        if (Wire.endTransmission() == 0) {
            int r = 0;
            Wire.requestFrom(devAddr, n);
            while (Wire.available() > 0 && r < n) {
                data[offset + r] = (byte)Wire.read();
                r++;
            }
        }
    }
}

int writeFloat(int devAddr, unsigned int writeAddress, float data)
{
    byte writebuf[4];
    memcpy(writebuf, (byte*)(&data), 4);
    write(devAddr, writeAddress, writebuf, 4);
    Serial.printf("%#4x - %#4x - %#4x - %#4x\t::\t%f\n", writebuf[0], writebuf[1], writebuf[2], writebuf[3], data);
    return 0;
}

float readFloat(int devAddr, unsigned int readAddress)
{
    byte buffer[4];
    read(devAddr, readAddress, buffer, 4);
    return *(float*)&buffer;
}

void writeCalibration(int devAddr, Octo_SlimeVR::Configuration::CalibrationConfig calibration)
{
    // Writing OctoSlime signature
    write(devAddr, signatureAddress, (byte*)signatureValue, 10);

    // Writing UNIX time of calibration
    byte time[4];
    unsigned long utime = ServerConnection::getUnixTime();
    memcpy(time, (byte*)(&utime), 4);
    write(devAddr, calDateAddress, time, 4);
    memset(time, 0, 4);
    read(devAddr, calDateAddress, time, 4);
    Serial.printf("read time : %lu\n", *(unsigned long*)&time);

    // Offset write
    for (int i = 0; i < 3; i++) {
        // Float has 4 bytes so address needs to increase by 4 for each float
        Serial.printf("writing offset float %d at %#2x: ", i, magOffsAddress + (i*4));
        writeFloat(devAddr, magOffsAddress + (i * 4), calibration.data.mpu9250.M_B[i]);
    }

    // Calibration matrix write
    int it = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            // Float has 4 bytes so address needs to increase by 4 for each float
            Serial.printf("writing calibration matrix float %d at %#2x: ", it, magCorrAddress + (it * 4));
            writeFloat(devAddr, magCorrAddress + (it * 4), calibration.data.mpu9250.M_Ainv[i][j]);
            it = it + 1;
        }
    }
    byte test[0x3e];
    read(devAddr, 0x00, test, 0x3e);
    Serial.printf("Read eeprom: ");
    for (int i = 0; i < 0x3e; i++) {
        if (i == magOffsAddress) {
            Serial.printf(" \nCALIBRATION DATA :::: -");
        }
        if (i - magOffsAddress >= 0 && (i - magOffsAddress) % 4 == 0) {
            Serial.printf(" \nnext float %#2x: ", i);
        }
        Serial.printf("%#2x - ", test[i]);
    }
    Serial.println();
}

void readCalibration(int devAddr, Octo_SlimeVR::Configuration::CalibrationConfig* calibration)
{
    Serial.println();

    // Reading UNIX time of calibration
    byte buff[4];
    read(devAddr, calDateAddress, buff, 4);
    calibration->data.mpu9250.date = *(unsigned long*)&buff;
    Serial.printf("Date: %d \n", calibration->data.mpu9250.date);
    float read;

    // Offset read
    for (int i = 0; i < 3; i++) {
        read = readFloat(devAddr, magOffsAddress + (i * 4));
        calibration->data.mpu9250.M_B[i] = read;
        Serial.printf("Read offset %d : %f\n", i, read);
        read = 0;
    }

    // Calibration matrix read
    int it = 0;
    for (int i = 0; i < 3; i++) {
        Serial.print("Read calibration matrix: ");
        for (int j = 0; j < 3; j++) {
            read = readFloat(devAddr, magCorrAddress + (it * 4));
            calibration->data.mpu9250.M_Ainv[i][j] = read;
            Serial.printf("%d : %f - ", it, read);
            read = 0;
            it += 1;
        }
        Serial.println();
    }
}

void clearCalibration(int devAddr)
{
    for (int i = 0; i < 0x40; i++) {
        writeByte(devAddr, i, (byte)0);
    }
}

boolean checkForCalibration(int devAddr)
{
    byte buff[10];
    read(devAddr, signatureAddress, buff, 10);
    buff[9] = '\0';
    Serial.printf("\n%s - %d - %s\n", buff, buff[9] == ((byte*)signatureValue)[9], signatureValue);
    return buff == (byte*)signatureValue;
}

unsigned long getDate(int devAddr)
{
    byte buff[4];
    read(devAddr, calDateAddress, buff, 4);
    return *(unsigned long*)&buff;
}

void test()
{
    int devAddr = eepromBankAddressA;

    clearCalibration(devAddr);

    // delay(5000);
    Serial.println("writing signature");
    write(devAddr, signatureAddress, (byte*)signatureValue, 10);

    byte time[4];
    unsigned long utime = 7456812;
    memcpy(time, (byte*)(&utime), 4);
    write(devAddr, calDateAddress, time, 4);

    // // Serial.println("got time");
    // Serial.printf("Write time : %#04x - %#04x - %#04x - %#04x \n", time[0], time[1], time[2], time[3]);
    // memset(time, 0, 4);
    // read(devAddr, calDateAddress, time, 4);
    // Serial.printf("read time : %d\n", *(unsigned long*)&time);

    float f = 4152.49257;
    writeFloat(devAddr, magOffsAddress, f);

    byte test[32];
    read(devAddr, 0x00, test, 32);
    Serial.printf("Read eeprom: ");
    for (int i = 0; i < 32; i++) {
        if (i == calDateAddress) {
            Serial.printf("- Cal date address - ");
        }
        Serial.printf("%#2x - ", test[i]);
        if (i == magOffsAddress - 1) {
            Serial.printf("cal date end - - ");
        }
    }
    Serial.println();
    byte buff[10];
    read(devAddr, signatureAddress, buff, 10);
    Serial.printf("%s\n", buff);
    memset(time, 0, 4);
    read(devAddr, calDateAddress, time, 4);
    Serial.printf("%lu\n", *(unsigned long*)&time);

    float readf = readFloat(devAddr, magOffsAddress);
    printf("%f\n", readf);

    // Serial.println("done");
}

}
