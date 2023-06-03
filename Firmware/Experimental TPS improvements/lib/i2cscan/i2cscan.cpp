#include "i2cscan.h"
#include "I2Cdev.h"

#ifdef ESP8266
uint8_t portArray[] = {16, 5, 4, 2, 14, 12, 13};
String portMap[] = {"D0", "D1", "D2", "D4", "D5", "D6", "D7"};
#elif defined(ESP32)
uint8_t portArray[] = {4, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
String portMap[] = {"4", "13", "14", "15", "16", "17", "18", "19", "21", "22", "23", "25", "26", "27", "32", "33"};
#endif

namespace I2CSCAN
{

    uint8_t buffer[1] = {0};

    struct IMU
    {
        String Name;
        uint8_t Address;
        uint8_t AltAddress;
        uint8_t WhoAmI;
        uint8_t WhoAmIRegister;

        IMU() {}

        IMU(String Name_, uint8_t Address_, uint8_t AltAddress_, uint8_t WhoAmI_, uint8_t WhoAmIRegister_)
        {
            Name = Name_;
            Address = Address_;
            AltAddress = AltAddress_;
            WhoAmI = WhoAmI_;
            WhoAmIRegister = WhoAmIRegister_;
        }
    };

    IMU Sensors[]
    {
        // IMU_TypeID,First Address, Second Address, WhoAmI return Value, WhoAmI register
        IMU("MPU6050", 0x68, 0x69, 0x68, 0x75),
        IMU("ICM-20948", 0x68, 0x69, 0x40, 0x00),
        IMU("BNO055", 0x4A, 0x4B, 0x14, 0x00),

    };

    uint8_t ReadReg(uint8_t addr, uint8_t regaddr)
    {
        buffer[0] = 0;
        I2Cdev::readByte(addr, regaddr, buffer);
        return buffer[0];
    }

    DeviceParams pickDevice(uint8_t AddressBank)
    {

        DeviceParams Retval;
        for (uint8_t i = 0; i < sizeof(Sensors) / sizeof(Sensors[0]); i++)
        {
            auto imu = Sensors[i];
            switch (AddressBank)
            {
            case 0:
                if (I2CSCAN::isI2CExist(imu.Address))
                {
                    Retval.DeviceAddress = imu.Address;
                    Retval.DeviceID = ReadReg(imu.Address, imu.WhoAmIRegister);
                    return Retval;
                }
                break;
            case 1:
                if (I2CSCAN::isI2CExist(imu.AltAddress))
                {
                    Retval.DeviceAddress = imu.AltAddress;
                    Retval.DeviceID = ReadReg(imu.AltAddress, imu.WhoAmIRegister);
                    return Retval;
                }
                break;

            default:
                Serial.println(F("Invalid Bank"));
                Retval.DeviceAddress = 0;
                Retval.DeviceID = 0;
                return Retval;
                break;
            }
        }

        Retval.DeviceAddress = 0;
        Retval.DeviceID = 0;
        return Retval;
    };

    bool isI2CExist(uint8_t addr)
    {
        Wire.beginTransmission(addr);
        byte error = Wire.endTransmission();
        if (error == 0)
            return true;
        return false;
    }

    /**
     * This routine turns off the I2C bus and clears it
     * on return SCA and SCL pins are tri-state inputs.
     * You need to call Wire.begin() after this to re-enable I2C
     * This routine does NOT use the Wire library at all.
     *
     * returns 0 if bus cleared
     *         1 if SCL held low.
     *         2 if SDA held low by slave clock stretch for > 2sec
     *         3 if SDA held low after 20 clocks.
     * From: http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html
     * (c)2014 Forward Computing and Control Pty. Ltd.
     * NSW Australia, www.forward.com.au
     * This code may be freely used for both private and commerical use
     */
    int clearBus(uint8_t SDA, uint8_t SCL)
    {
#if defined(TWCR) && defined(TWEN)
        TWCR &= ~(_BV(TWEN)); // Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

        pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
        pinMode(SCL, INPUT_PULLUP);

        boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
        if (SCL_LOW)
        {             // If it is held low Arduno cannot become the I2C master.
            return 1; // I2C bus error. Could not clear SCL clock line held low
        }

        boolean SDA_LOW = (digitalRead(SDA) == LOW); // vi. Check SDA input.
        int clockCount = 20;                         // > 2x9 clock

        while (SDA_LOW && (clockCount > 0))
        { //  vii. If SDA is Low,
            clockCount--;
            // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
            pinMode(SCL, INPUT);        // release SCL pullup so that when made output it will be LOW
            pinMode(SCL, OUTPUT);       // then clock SCL Low
            delayMicroseconds(10);      //  for >5uS
            pinMode(SCL, INPUT);        // release SCL LOW
            pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
            // do not force high as slave may be holding it low for clock stretching.
            delayMicroseconds(10); //  for >5uS
            // The >5uS is so that even the slowest I2C devices are handled.
            SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
            int counter = 20;
            while (SCL_LOW && (counter > 0))
            { //  loop waiting for SCL to become High only wait 2sec.
                counter--;
                delay(100);
                SCL_LOW = (digitalRead(SCL) == LOW);
            }
            if (SCL_LOW)
            {             // still low after 2 sec error
                return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
            }
            SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
        }
        if (SDA_LOW)
        {             // still low
            return 3; // I2C bus error. Could not clear. SDA data line held low
        }

        // else pull SDA line low for Start or Repeated Start
        pinMode(SDA, INPUT);  // remove pullup.
        pinMode(SDA, OUTPUT); // and then make it LOW i.e. send an I2C Start or Repeated start control.
        // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
        /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
        delayMicroseconds(10);      // wait >5uS
        pinMode(SDA, INPUT);        // remove output low
        pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
        delayMicroseconds(10);      // x. wait >5uS
        pinMode(SDA, INPUT);        // and reset pins as tri-state inputs which is the default state on reset
        pinMode(SCL, INPUT);
        return 0; // all ok
    }

}
