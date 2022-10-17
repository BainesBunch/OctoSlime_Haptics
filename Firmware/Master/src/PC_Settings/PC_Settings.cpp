#include <PC_Settings\PC_Settings.h>

namespace PC_Settings
{
    // ===================================================================================
// =								Comms Handshake values							 =
// ===================================================================================
#define ASCII_STX 2
#define ASCII_ETX 3
#define ASCII_ENQ 5
#define ASCII_ACK 6

    boolean Retval = false;
    char Inchar;
    uint16 CommandPointer = 0;
    char CommandIn[70];
    char PayloadValue[70];
    char OPBuff[100];

    boolean CheckForPCCommands()
    {
        Retval = false;
        if (Serial.available() > 0)
        {
            while (Serial.available() > 0)
            {
                Inchar = Serial.read();

                if ((int)Inchar > 31 || (char)Inchar == '\r' || (char)Inchar == '\n')
                {
                    if ((char)Inchar == '\r' || (char)Inchar == '\n')
                    {
                        if (CommandPointer > 0)
                        {
                            PC_Settings::ProcessPCResponseString();
                        }
                        CommandPointer = 0;
                        while (Serial.available() > 0)
                            Serial.read();
                        break;
                    }
                    else
                    {
                        if (CommandPointer < 50)
                            CommandIn[CommandPointer++] = ((char)Inchar);
                    }
                }
                if (Serial.available() == 0)
                    delay(50);
            }
            Retval = true;
        }
        return Retval;
    }

    void ProcessPCResponseString()
    {

        memset(PayloadValue, 0, 70);

        if (CommandPointer > 1)
        {
            for (uint8_t cLoop = 2; cLoop < CommandPointer; cLoop++)
            {
                PayloadValue[cLoop - 2] = CommandIn[cLoop];
            }
        }

        Serial.write(byte(ASCII_ENQ));

        switch (CommandIn[0])
        {

        case 'Q':
           // UI::WriteMessage("Data Request");
            PC_Settings::SendConfigData();
            CommandPointer = 0;
            return;
            break;

        case 'S':
            //UI::WriteMessage("Setting SSID");
            Octo_SlimeVR::Configuration::SetSSID(PayloadValue);
            break;

        case 'P':
           // UI::WriteMessage("Setting Password");
            Octo_SlimeVR::Configuration::SetPassword(PayloadValue);
            break;

        case 'H':
            //UI::WriteMessage("Setting Haptic Flag");
            Octo_SlimeVR::Configuration::SetHapticsEnabled(PayloadValue);
            break;
        }

        Serial.write((byte)ASCII_ETX);
        delay(500);
        Serial.flush();
    }

    void SendConfigData(void)
    {
        Serial.print(F("Q"));

        Serial.print(F("S,"));

        Serial.print(F(";P,"));

        Serial.print(F(";H,"));

        Serial.print(F("\r\n"));
        Serial.flush();
    }

}