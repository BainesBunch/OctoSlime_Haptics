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
        else
        {
            uint8_t TempMask = Haptics_Branches[MuxID].Controler_State[ControlerID].CurrentBitmaskValue;
            if (Level > 0)
            {
                REG_SET_BIT(TempMask, OutputID);
            }
            else
            {
                REG_CLR_BIT(TempMask, OutputID);
            }

            Wire.beginTransmission(uint8_t(OnOff_Base_Address + ControlerID));
            Wire.write(0x44);
            Wire.write(TempMask);
            Wire.endTransmission();

            Haptics_Branches[MuxID].Controler_State[ControlerID].CurrentBitmaskValue = TempMask;
        }
    }

    void Discovery()
    {
        for (uint8_t MuxID = 0; MuxID < 7; MuxID++)
        {

            for (uint8_t ControlerID = 0; ControlerID < 7; ControlerID++)
            {

                Haptics_Server_Registered[(MuxID * 8) + ControlerID] = 0;

                Haptics_Branches[MuxID].MuxID = MuxID;
                Haptics_Branches[MuxID].Controler_State[ControlerID].Haptic_DeviceType = None;
                Haptics_Branches[MuxID].Controler_State[ControlerID].CurrentBitmaskValue = 0;
                for (uint8_t ValueID = 0; ValueID < 7; ValueID++)
                {

                    Haptics_Branches[MuxID].Controler_State[ControlerID].CurrentPWMValue[ValueID] = 0;
                }
                // test for PWM devices first
                Wire.beginTransmission(PWM_Base_Address + ControlerID);
                if (Wire.endTransmission() == 0)
                {
                    Haptics_Branches[MuxID].Controler_State[ControlerID].Haptic_DeviceType = PWM;
                }
                else
                {
                    // test for OnOff devices Next
                    Wire.beginTransmission(OnOff_Base_Address + ControlerID);
                    if (Wire.endTransmission() == 0)
                    {
                        Haptics_Branches[MuxID].Controler_State[ControlerID].Haptic_DeviceType = OnOff;
                    }
                }
                Haptics_Branches[MuxID].Controler_State[ControlerID].Address = ControlerID;
            }
        }
    }
}
