#include <Haptics/Haptics.h>
#include "I2Cdev.h"

namespace Haptics
{

    uint8_t PWM_Base_Address = 0x30;
    uint8_t OnOff_Base_Address = 0x60;
    
     void SetLevel(uint8_t MuxID, uint8_t ControlerID, uint8_t OutputID, uint8_t Level)
    {
        I2Cdev::SetIMU(MuxID);

        if (GetHapticDeviceType(MuxID,ControlerID) == PWM)
        {

            if (GetHapticPWM(MuxID,ControlerID,OutputID) != Level)
            {
                Wire.beginTransmission(uint8_t(PWM_Base_Address + ControlerID));
                Wire.write(0x88);
                Wire.write(OutputID);
                Wire.write(Level);
                Wire.endTransmission();
                SetHapticPWM(MuxID,ControlerID,OutputID,Level);
            }
        }

        if (GetHapticDeviceType(MuxID,ControlerID) == OnOff)
        {
            uint8_t TempMask = GetHapticBitmask(MuxID,ControlerID);

            if (Level > 0)
            {
                TempMask |= (0x01 << OutputID);
            }
            else
            {
                TempMask &= ~(0x01 << OutputID);
            }

            Wire.beginTransmission(uint8_t(OnOff_Base_Address + ControlerID));
            Wire.write(0x44);
            Wire.write(TempMask);
            Wire.endTransmission();

            SetHapticBitmask(MuxID,ControlerID,TempMask);
        }
    }

    void Discovery()
    {
        Serial.println(F("Haptics Begin Scan"));
        for (uint8_t MuxID = 0; MuxID < 7; MuxID++)
        {

            Serial.print(F("Setting Mux : "));
            Serial.println(MuxID);

            I2Cdev::SetIMU(MuxID);

            for (uint8_t ControlerID = 0; ControlerID < 8; ControlerID++)
            {

                //   Serial.println("Setting Register Byte");
                SetRegisteredState(MuxID, ControlerID, 0);

                //  Serial.println("Setting Mux Byte");
                Haptics_Branches[MuxID].MuxID = MuxID;
                //  Serial.println("Setting Controler Type to None");
                SetHapticDeviceType(MuxID, ControlerID, None);
                //   Serial.println("Setting Bitmask to zero");
                SetHapticBitmask(MuxID, ControlerID, 0);

                //   Serial.println("Setting PWM Bytes to zero");
                for (uint8_t MotorID = 0; MotorID < 8; MotorID++)
                {
                    SetHapticPWM(MuxID, ControlerID, MotorID, 0);
                }

                // test for PWM devices first

                Wire.beginTransmission(PWM_Base_Address + ControlerID);
                if (Wire.endTransmission() == 0)
                {
                    SetHapticDeviceType(MuxID, ControlerID, PWM);
                    Serial.print(F("Found PWM at address : "));
                    Serial.println(PWM_Base_Address + ControlerID, HEX);
                }

                // test for OnOff devices Next
                Wire.beginTransmission(OnOff_Base_Address + ControlerID);
                if (Wire.endTransmission() == 0)
                {
                    SetHapticDeviceType(MuxID, ControlerID, OnOff);
                    Serial.print(F("Found OnOff at address : "));
                    Serial.println(OnOff_Base_Address + ControlerID, HEX);
                }

                SetHapticControlerID(MuxID, ControlerID);
            }
        }
        Serial.println(F("Haptics End Scan"));
    }

    void SetHapticDeviceType(uint8_t MuxID, uint8_t ControlerID, Haptic_DeviceType_e DeviceType)
    {
        Haptics_Mux_Branch_Devices_t Branch = Haptics_Branches[MuxID];
        Haptics_Controler_State_t Controler = Branch.Controler_State[ControlerID];
        Controler.Haptic_DeviceType = DeviceType;
        Branch.Controler_State[ControlerID] = Controler;
        Haptics_Branches[MuxID] = Branch;
    }

    uint8_t GetHapticDeviceType(uint8_t MuxID, uint8_t ControlerID)
    {
        Haptics_Mux_Branch_Devices_t Branch = Haptics_Branches[MuxID];
        Haptics_Controler_State_t Controler = Branch.Controler_State[ControlerID];
        return Controler.Haptic_DeviceType;
    }

    void SetHapticControlerID(uint8_t MuxID, uint8_t ControlerID)
    {
        Haptics_Mux_Branch_Devices_t Branch = Haptics_Branches[MuxID];
        Haptics_Controler_State_t Controler = Branch.Controler_State[ControlerID];
        Controler.Address = ControlerID;
        Branch.Controler_State[ControlerID] = Controler;
        Haptics_Branches[MuxID] = Branch;
    }

    void SetHapticPWM(uint8_t MuxID, uint8_t ControlerID, uint8_t Motor, uint8_t Level)
    {
        Haptics_Mux_Branch_Devices_t Branch = Haptics_Branches[MuxID];
        Haptics_Controler_State_t Controler = Branch.Controler_State[ControlerID];
        Controler.CurrentPWMValue[Motor] = Level;
        Branch.Controler_State[ControlerID] = Controler;
        Haptics_Branches[MuxID] = Branch;
    }

    uint8_t GetHapticPWM(uint8_t MuxID, uint8_t ControlerID, uint8_t Motor)
    {
        Haptics_Mux_Branch_Devices_t Branch = Haptics_Branches[MuxID];
        Haptics_Controler_State_t Controler = Branch.Controler_State[ControlerID];
        return Controler.CurrentPWMValue[Motor];
    }

    void SetHapticBitmask(uint8_t MuxID, uint8_t ControlerID, uint8_t Level)
    {
        Haptics_Mux_Branch_Devices_t Branch = Haptics_Branches[MuxID];
        Haptics_Controler_State_t Controler = Branch.Controler_State[ControlerID];
        Controler.CurrentBitmaskValue = Level;
        Branch.Controler_State[ControlerID] = Controler;
        Haptics_Branches[MuxID] = Branch;
    }

    uint8_t GetHapticBitmask(uint8_t MuxID, uint8_t ControlerID)
    {
        Haptics_Mux_Branch_Devices_t Branch = Haptics_Branches[MuxID];
        Haptics_Controler_State_t Controler = Branch.Controler_State[ControlerID];
        return Controler.CurrentBitmaskValue;
    }

    void SetRegisteredState(uint8_t MuxID, uint8_t ControlerID, uint8_t State)
    {
        Haptics_Server_Registered[(MuxID * 8) + ControlerID] = State;
    }

    uint8_t GetRegisteredState(uint8_t MuxID, uint8_t ControlerID)
    {
        return Haptics_Server_Registered[(MuxID * 8) + ControlerID];
    }

}
