#include <Haptics/Haptics.h>
#include "I2Cdev.h"

namespace Haptics
{

    uint8_t PWM_Base_Address = 0x30;
    uint8_t OnOff_Base_Address = 0x60;


    void SetLevel(uint8_t MuxID, uint8_t ControlerID, uint8_t OutputID, uint8_t Level)
    {
        I2Cdev::SetIMU(MuxID);

        if (Haptics_Branches[MuxID].Controler_State[ControlerID].Haptic_DeviceType == PWM)
        {

            if (Haptics_Branches[MuxID].Controler_State[ControlerID].CurrentPWMValue[OutputID] != Level)
            {
                Wire.beginTransmission(uint8_t(PWM_Base_Address + ControlerID));
                Wire.write(OutputID);
                Wire.write(Level);
                Wire.endTransmission();
                Haptics_Branches[MuxID].Controler_State[ControlerID].CurrentPWMValue[OutputID] = Level;
            }
        }
        
        if (Haptics_Branches[MuxID].Controler_State[ControlerID].Haptic_DeviceType == OnOff)
        {
            uint8_t TempMask = Haptics_Branches[MuxID].Controler_State[ControlerID].CurrentBitmaskValue;

            if (Level > 0)
            {
                TempMask |= (0x01 << OutputID);
            }
            else
            {
                TempMask &= ~(0x01 << OutputID);
            }


            // Serial.print("Bitmask : ");

            // for (uint8_t i = 0; i < 8; i++)  
            // {
            //     Serial.print(((TempMask >> i) & 1) == 1 ? "1" : "0");
            // }
            // Serial.println();
            
            Wire.beginTransmission(uint8_t(OnOff_Base_Address + ControlerID));
            Wire.write(0x44);
            Wire.write(TempMask);
            Wire.endTransmission();

            Haptics_Branches[MuxID].Controler_State[ControlerID].CurrentBitmaskValue = TempMask;
        }
    }




    void Discovery()
    {
//        Serial.println("Haptics Begin Scan");
        for (uint8_t MuxID = 0; MuxID < 7; MuxID++)
        {
            //Serial.print("Setting Mux : ");
            //Serial.println(MuxID);

            I2Cdev::SetIMU(MuxID);

            for (uint8_t ControlerID = 0; ControlerID < 8; ControlerID++)
            {

             //   Serial.println("Setting Register Byte");
                Haptics_Server_Registered[(MuxID * 8) + ControlerID] = 0;

              //  Serial.println("Setting Mux Byte");
                Haptics_Branches[MuxID].MuxID = MuxID;
             //  Serial.println("Setting Controler Type to None");
                Haptics_Branches[MuxID].Controler_State[ControlerID].Haptic_DeviceType = None;
             //   Serial.println("Setting Bitmask to zero");
                Haptics_Branches[MuxID].Controler_State[ControlerID].CurrentBitmaskValue = 0;

             //   Serial.println("Setting PWM Bytes to zero");
                for (uint8_t ValueID = 0; ValueID < 7; ValueID++)
                {
                    Haptics_Branches[MuxID].Controler_State[ControlerID].CurrentPWMValue[ValueID] = 0;
                }
                // test for PWM devices first
              //  Serial.print("Scanning at address : ");
              //
              
                Serial.println(PWM_Base_Address + ControlerID,HEX);
                Wire.beginTransmission(PWM_Base_Address + ControlerID);
                if (Wire.endTransmission() == 0)
                {
                    Haptics_Branches[MuxID].Controler_State[ControlerID].Haptic_DeviceType = PWM;
                    Serial.println("Found PWM");
                }
                // test for OnOff devices Next
             //   Serial.print("Scanning at address : ");
             //   Serial.println(OnOff_Base_Address + ControlerID,HEX);
                Wire.beginTransmission(OnOff_Base_Address + ControlerID);
                if (Wire.endTransmission() == 0)
                {
                    Haptics_Branches[MuxID].Controler_State[ControlerID].Haptic_DeviceType = OnOff;
                    Serial.println("Found OnOff");
                }
            Haptics_Branches[MuxID].Controler_State[ControlerID].Address = ControlerID;
            }
        }
        //Serial.println("Haptics End Scan");
    }
}
