#ifndef SLIMEVR_HAPTICS_H_
#define SLIMEVR_HAPTICS_H_

#include <Arduino.h>
#include "globals.h"
#include <Wire.h>
#include "I2Cdev.h"

namespace Haptics
{

    typedef enum
    {
        None = 0,
        OnOff = 1,
        PWM = 2

    } Haptic_DeviceType_e;

    struct Haptics_Controler_State_t
    {
        uint8_t Haptic_DeviceType;
        uint8_t CurrentPWMValue[8];
        uint8_t CurrentBitmaskValue;
        uint8_t DeviceID;
        uint8_t Address;
    };

    struct Haptics_Mux_Branch_Devices_t
    {
        Haptics_Controler_State_t Controler_State[8];
        uint8_t MuxID;
    };

    static Haptics_Mux_Branch_Devices_t Haptics_Branches[8];
    static uint8_t Haptics_Server_Registered[64];

    void SetHapticControlerID(uint8_t MuxID, uint8_t ControlerID);

    void SetHapticDeviceType(uint8_t MuxID, uint8_t ControlerID, Haptic_DeviceType_e DeviceType);
    uint8_t GetHapticDeviceType(uint8_t MuxID, uint8_t ControlerID);

    void SetRegisteredState(uint8_t MuxID, uint8_t ControlerID, uint8_t State);
    uint8_t GetRegisteredState(uint8_t MuxID, uint8_t ControlerID);
    
    void SetHapticPWM(uint8_t MuxID, uint8_t ControlerID, uint8_t Motor, uint8_t Level);
    uint8_t GetHapticPWM(uint8_t MuxID, uint8_t ControlerID, uint8_t Motor);

    void SetHapticBitmask(uint8_t MuxID, uint8_t ControlerID,uint8_t Level);
    uint8_t GetHapticBitmask(uint8_t MuxID, uint8_t ControlerID);

    void SetLevel(uint8_t MuXID, uint8_t ControlerID, uint8_t OutputID, uint8_t Level);
    void Discovery();

 
}

#endif // SLIMEVR_HAPTICS_H_