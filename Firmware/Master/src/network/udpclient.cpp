/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "udpclient.h"
#include "packets.h"
#include <NTPClient.h>

#define TIMEOUT 3000UL

WiFiUDP Udp;
WiFiUDP Haptics_Udp;

NTPClient timeClient(Udp, "pool.ntp.org");

unsigned char incomingPacket[128]; // buffer for incoming packets
uint64_t packetNumber = 0;
unsigned char handshake[12] = {0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0};

int port = 6969;
int Haptics_port = 2022;

IPAddress host = IPAddress(255, 255, 255, 255);
IPAddress Haptics_host = IPAddress(255, 255, 255, 255);

unsigned long lastConnectionAttemptMs;
unsigned long lastPacketMs;

unsigned long lastHapticsConnectionAttemptMs;
unsigned long lastHapticsPacketMs;

bool connected = false;
bool Haptics_connected = false;

uint8_t sensorStateNotifieds[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned long lastSensorInfoPacket = 0;

unsigned long lastHapticSensorInfoPacket = 0;

unsigned long LastIncommingMessageTime = 0;
unsigned long LastIncommingHapticMessageTime = 0;

uint8_t serialBuffer[128];
size_t serialLength = 0;

unsigned char buf[8];

template <typename T>
unsigned char *convert_to_chars(T src, unsigned char *target)
{
    union uwunion
    {
        unsigned char c[sizeof(T)];
        T v;
    } un;
    un.v = src;
    for (unsigned int i = 0; i < sizeof(T); i++)
    {
        target[i] = un.c[sizeof(T) - i - 1];
    }
    return target;
}

template <typename T>
T convert_chars(unsigned char *const src)
{
    union uwunion
    {
        unsigned char c[sizeof(T)];
        T v;
    } un;
    for (unsigned int i = 0; i < sizeof(T); i++)
    {
        un.c[i] = src[sizeof(T) - i - 1];
    }
    return un.v;
}

namespace DataTransfer
{

    bool beginPacket()
    {
        int r = Udp.beginPacket(host, port);
        if (r == 0)
        {
            // Print error
        }
        return r > 0;
    }

    bool beginHapticPacket()
    {
        int r = Haptics_Udp.beginPacket(Haptics_host, Haptics_port);
        if (r == 0)
        {
            // Print error
        }
        return r > 0;
    }

    bool endPacket()
    {
        int r = Udp.endPacket();
        if (r == 0)
        {
            // Print error
        }
        return r > 0;
    }

    bool endHapticPacket()
    {
        int r = Haptics_Udp.endPacket();
        if (r == 0)
        {
            // Print error
        }
        LastIncommingMessageTime = millis();
        return r > 0;
    }

    void sendPacketType(uint8_t type)
    {
        Udp.write(0);
        Udp.write(0);
        Udp.write(0);
        Udp.write(type);
    }

    void sendHapticPacketType(uint8_t type)
    {
        Haptics_Udp.write(0);
        Haptics_Udp.write(0);
        Haptics_Udp.write(0);
        Haptics_Udp.write(type);
    }

    void sendPacketNumber()
    {
        uint64_t pn = packetNumber++;
        sendLong(pn);
    }

    void sendFloat(float f)
    {
        Udp.write(convert_to_chars(f, buf), sizeof(f));
    }

    void sendByte(uint8_t c)
    {
        Udp.write(&c, 1);
    }

    void sendHapticByte(uint8_t c)
    {
        Haptics_Udp.write(&c, 1);
    }

    void sendInt(int i)
    {
        Udp.write(convert_to_chars(i, buf), sizeof(i));
    }

    void sendLong(uint64_t l)
    {
        Udp.write(convert_to_chars(l, buf), sizeof(l));
    }

    void sendBytes(const uint8_t *c, size_t length)
    {
        Udp.write(c, length);
    }

    void sendHapticBytes(const uint8_t *c, size_t length)
    {
        Haptics_Udp.write(c, length);
    }

    void sendShortString(const char *str)
    {
        uint8_t size = strlen(str);
        sendByte(size);                        // String size
        sendBytes((const uint8_t *)str, size); // Firmware version string
    }

    void sendLongString(const char *str)
    {
        int size = strlen(str);
        sendInt(size);                         // String size
        sendBytes((const uint8_t *)str, size); // Firmware version string
    }

    int getWriteError()
    {
        return Udp.getWriteError();
    }

    int getHapticWriteError()
    {
        return Haptics_Udp.getWriteError();
    }
}

// PACKET_HEARTBEAT 0
void Network::sendHeartbeat()
{
    if (DataTransfer::beginPacket())
    {
        DataTransfer::sendPacketType(PACKET_HEARTBEAT);
        DataTransfer::sendPacketNumber();
        DataTransfer::endPacket();
    }
}

void Network::sendHapticHeartbeat()
{
    if (DataTransfer::beginHapticPacket())
    {
        DataTransfer::sendHapticPacketType(PACKET_OCTOSLIME_HAPTICS_HEARTBEAT);
        DataTransfer::endHapticPacket();
    }
}

// PACKET_ACCEL 4
void Network::sendAccel(float *vector, uint8_t sensorId)
{
    if (DataTransfer::beginPacket())
    {
        DataTransfer::sendPacketType(PACKET_ACCEL);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendFloat(vector[0]);
        DataTransfer::sendFloat(vector[1]);
        DataTransfer::sendFloat(vector[2]);
        DataTransfer::endPacket();
    }
}

// PACKET_RAW_CALIBRATION_DATA 6
void Network::sendRawCalibrationData(float *vector, uint8_t calibrationType, uint8_t sensorId)
{
    if (DataTransfer::beginPacket())
    {
        DataTransfer::sendPacketType(PACKET_RAW_CALIBRATION_DATA);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(sensorId);
        DataTransfer::sendInt(calibrationType);
        DataTransfer::sendFloat(vector[0]);
        DataTransfer::sendFloat(vector[1]);
        DataTransfer::sendFloat(vector[2]);
        DataTransfer::endPacket();
    }
}

void Network::sendRawCalibrationData(int *vector, uint8_t calibrationType, uint8_t sensorId)
{
    if (DataTransfer::beginPacket())
    {
        DataTransfer::sendPacketType(PACKET_RAW_CALIBRATION_DATA);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(sensorId);
        DataTransfer::sendInt(calibrationType);
        DataTransfer::sendInt(vector[0]);
        DataTransfer::sendInt(vector[1]);
        DataTransfer::sendInt(vector[2]);
        DataTransfer::endPacket();
    }
}

// PACKET_CALIBRATION_FINISHED 7
void Network::sendCalibrationFinished(uint8_t calibrationType, uint8_t sensorId)
{
    if (DataTransfer::beginPacket())
    {
        DataTransfer::sendPacketType(PACKET_CALIBRATION_FINISHED);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(sensorId);
        DataTransfer::sendInt(calibrationType);
        DataTransfer::endPacket();
    }
}

// PACKET_BATTERY_LEVEL 12
void Network::sendBatteryLevel(float batteryVoltage, float batteryPercentage)
{
    if (DataTransfer::beginPacket())
    {
        DataTransfer::sendPacketType(PACKET_BATTERY_LEVEL);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendFloat(batteryVoltage);
        DataTransfer::sendFloat(batteryPercentage);
        DataTransfer::endPacket();
    }
}

// PACKET_TAP 13
void Network::sendTap(uint8_t value, uint8_t sensorId)
{
    if (DataTransfer::beginPacket())
    {
        DataTransfer::sendPacketType(PACKET_TAP);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(sensorId);
        DataTransfer::sendByte(value);
        DataTransfer::endPacket();
    }
}

// PACKET_ERROR 14
void Network::sendError(uint8_t reason, uint8_t sensorId)
{
    if (DataTransfer::beginPacket())
    {
        DataTransfer::sendPacketType(PACKET_ERROR);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(sensorId);
        DataTransfer::sendByte(reason);
        DataTransfer::endPacket();
    }
}

// PACKET_SENSOR_INFO 15
void Network::sendSensorInfo(Sensor *sensor)
{
    if (DataTransfer::beginPacket())
    {
        DataTransfer::sendPacketType(PACKET_SENSOR_INFO);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(sensor->getSensorId());
        DataTransfer::sendByte(sensor->getSensorState());
        DataTransfer::sendByte(sensor->getSensorType());
        DataTransfer::endPacket();
    }
}

// PACKET_OCTOSLIME_HAPTICS_INFO 0x71
void Network::sendHapticsInfo(uint8_t MuxID, uint8_t ControlerID, uint8_t DeviceTypeID)
{
    if (DataTransfer::beginHapticPacket())
    {
        DataTransfer::sendHapticPacketType(PACKET_OCTOSLIME_HAPTICS_INFO);
        DataTransfer::sendHapticByte(MuxID);
        DataTransfer::sendHapticByte(ControlerID);
        DataTransfer::sendHapticByte(DeviceTypeID);
        DataTransfer::endHapticPacket();
    }
}

// PACKET_ROTATION_DATA 17
void Network::sendRotationData(Quat *const quaternion, uint8_t dataType, uint8_t accuracyInfo, uint8_t sensorId)
{
    if (DataTransfer::beginPacket())
    {
        DataTransfer::sendPacketType(PACKET_ROTATION_DATA);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(sensorId);
        DataTransfer::sendByte(dataType);
        DataTransfer::sendFloat(quaternion->x);
        DataTransfer::sendFloat(quaternion->y);
        DataTransfer::sendFloat(quaternion->z);
        DataTransfer::sendFloat(quaternion->w);
        DataTransfer::sendByte(accuracyInfo);
        DataTransfer::endPacket();

        // Serial.print(F("Sensor ID : "));
        // Serial.print(sensorId);

        // Serial.print(F(" X : "));
        // Serial.print(quaternion->x);

        // Serial.print(F(" Y : "));
        // Serial.print(quaternion->y);

        // Serial.print(F(" Z : "));
        // Serial.print(quaternion->z);

        // Serial.print(F(" W : "));
        // Serial.print(quaternion->w);

        // Serial.print(F(" Accuracy : "));
        // Serial.println(accuracyInfo);
    }
}

// PACKET_MAGNETOMETER_ACCURACY 18
void Network::sendMagnetometerAccuracy(float accuracyInfo, uint8_t sensorId)
{
    if (DataTransfer::beginPacket())
    {
        DataTransfer::sendPacketType(PACKET_MAGNETOMETER_ACCURACY);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(sensorId);
        DataTransfer::sendFloat(accuracyInfo);
        DataTransfer::endPacket();
    }
}

// PACKET_SIGNAL_STRENGTH 19
void Network::sendSignalStrength(uint8_t signalStrength)
{
    if (DataTransfer::beginPacket())
    {
        DataTransfer::sendPacketType(PACKET_SIGNAL_STRENGTH);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(255);
        DataTransfer::sendByte(signalStrength);
        DataTransfer::endPacket();
    }
}

// PACKET_TEMPERATURE 20
void Network::sendTemperature(float temperature, uint8_t sensorId)
{
    if (DataTransfer::beginPacket())
    {
        DataTransfer::sendPacketType(PACKET_TEMPERATURE);
        DataTransfer::sendPacketNumber();
        DataTransfer::sendByte(sensorId);
        DataTransfer::sendFloat(temperature);
        DataTransfer::endPacket();
    }
}

void Network::sendHapticsHandshake()
{
    if (DataTransfer::beginHapticPacket())
    {
        DataTransfer::sendHapticPacketType(PACKET_OCTOSLIME_HAPTICS_HANDSHAKE);
        uint8_t mac[6];
        WiFi.macAddress(mac);
        DataTransfer::sendHapticBytes(mac, 6); // MAC address string
        DataTransfer::endHapticPacket();
    }
    else
    {
        Serial.print(F("Haptics Server begin packet Error : "));
        Serial.println(DataTransfer::getHapticWriteError());
    }
}

void Network::sendHandshake()
{
    if (DataTransfer::beginPacket())
    {
        DataTransfer::sendPacketType(PACKET_HANDSHAKE);
        DataTransfer::sendLong(0); // Packet number is always 0 for handshake
        DataTransfer::sendInt(0);
        // This is kept for backwards compatibility,
        // but the latest SlimeVR server will not initialize trackers
        // with firmware build > 8 until it recieves sensor info packet
        DataTransfer::sendInt(0);
        DataTransfer::sendInt(HARDWARE_MCU);
        DataTransfer::sendInt(0);
        DataTransfer::sendInt(0);
        DataTransfer::sendInt(0);
        DataTransfer::sendInt(FIRMWARE_BUILD_NUMBER); // Firmware build number
        DataTransfer::sendShortString(FIRMWARE_VERSION);
        uint8_t mac[6];
        WiFi.macAddress(mac);
        DataTransfer::sendBytes(mac, 6); // MAC address string
        if (DataTransfer::endPacket())
        {
           // Serial.print(F("Handshake write error: "));
           // Serial.println(Udp.getWriteError());
        }
    }
    else
    {
       // Serial.print(F("Handshake write error: "));
       // Serial.println(Udp.getWriteError());
    }
}

void returnLastPacket(int len)
{
    if (DataTransfer::beginPacket())
    {
        DataTransfer::sendBytes(incomingPacket, len);
        DataTransfer::endPacket();
    }
}

void updateSensorState(Sensor *Sensors[])
{
    if (millis() - lastSensorInfoPacket > 1000)
    {
        lastSensorInfoPacket = millis();
        for (int BankCount = 0; BankCount < 2; BankCount++)
        {
            for (int SensorCount = 0; SensorCount < IMUCount; SensorCount++)
            {
                uint8_t SensorID = SensorCount + (BankCount * IMUCount);
                if (Sensors[SensorID]->Connected)
                {
                    if (Sensors[SensorID]->isWorking() && sensorStateNotifieds[SensorID] == 0)
                    {
                        Serial.print("Sending Sensor Register Info for Sensor ID : ");
                        Serial.println(SensorID);
                        Network::sendSensorInfo(Sensors[SensorID]);
                    }
                }
            }
        }
    }
}

void updateOctoSlimeHapticState()
{
    if (millis() - lastHapticSensorInfoPacket > 1000)
    {
        lastHapticSensorInfoPacket = millis();
        bool SentDiscoPacket = false;
        for (uint8_t MuxID = 0; MuxID < 8; MuxID++)
        {
            for (int ControlerID = 0; ControlerID < 8; ControlerID++)
            {
                //uint8_t SensorID = ControlerID + (MuxID * 8);
                if ((Haptics::GetHapticDeviceType(MuxID, ControlerID) != Haptics::None) && Haptics::GetRegisteredState(MuxID, ControlerID) == 0)
                {

                    // Serial.println("Sending Haptics Register Info for");
                    // Serial.print(" Sensor ID : ");
                    // Serial.print(SensorID);
                    // Serial.print(" Device Type : ");
                    // Serial.print(Haptics::GetHapticDeviceType(MuxID, ControlerID));
                    // Serial.print(" Registered : ");
                    // Serial.println(Haptics::GetRegisteredState(MuxID, ControlerID));
                    Network::sendHapticsInfo(MuxID, ControlerID, Haptics::GetHapticDeviceType(MuxID, ControlerID));
                    SentDiscoPacket = true;
                }
            }
        }
        if (SentDiscoPacket == false)
        {
           // Serial.println(F("Sending Heartbeat Token"));
            Network::sendHapticHeartbeat();
        }
    }
}

int ServerConnection::DataReady()
{

    return Haptics_Udp.parsePacket();
}

void ServerConnection::Hapticsconnect()
{
    unsigned long now = millis();
    while (true)
    {
        int packetSize = Haptics_Udp.parsePacket();
        if (packetSize)
        {
            LastIncommingMessageTime = millis();
            // receive incoming UDP packets
            // Serial.printf("[Haptic Handshake] Received %d bytes from %s, port %d\n", packetSize, Haptics_Udp.remoteIP().toString().c_str(), Haptics_Udp.remotePort());
            int len = Haptics_Udp.read(incomingPacket, sizeof(incomingPacket));
            // Serial.print("[Haptic Handshake] UDP packet contents: ");
            // for (int i = 0; i < len; ++i)
            //     Serial.print((byte)incomingPacket[i]);
            // Serial.println();
            // Handshake is different, it has 3 in the first byte, not the 4th, and data starts right after
            switch (incomingPacket[0])
            {
            case PACKET_OCTOSLIME_HAPTICS_RECEIVE_HANDSHAKE:
                // Assume handshake successful, don't check it
                // But proper handshake should contain "Hey OVR =D 5" ASCII string right after the packet number
                // Starting on 14th byte (packet number, 12 bytes greetings, null-terminator) we can transfer SlimeVR handshake data
                Haptics_host = Haptics_Udp.remoteIP();
                Haptics_port = Haptics_Udp.remotePort();
                lastPacketMs = now;
                Haptics_connected = true;

                Serial.printf("[Haptic Handshake] Handshake successful, Haptic server is %s:%d\n", Haptics_Udp.remoteIP().toString().c_str(), Haptics_Udp.remotePort());
                // UI::SetMessage(6);
                return;
            default:
                continue;
            }
        }
        else
        {
            break;
        }
    }
    if (lastHapticsConnectionAttemptMs + 1000 < now)
    {
        lastHapticsConnectionAttemptMs = now;
        Serial.println(F("Looking for the Haptic server..."));
        Network::sendHapticsHandshake();
    }
}

void ServerConnection::Slimeconnect()
{
    unsigned long now = millis();
    while (true)
    {
        int packetSize = Udp.parsePacket();
        if (packetSize)
        {
            // receive incoming UDP packets
            Serial.printf("[Handshake] Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
            int len = Udp.read(incomingPacket, sizeof(incomingPacket));
            Serial.print(F("[Handshake] UDP packet contents: "));
            for (int i = 0; i < len; ++i)
                Serial.print((byte)incomingPacket[i]);
            Serial.println();
            // Handshake is different, it has 3 in the first byte, not the 4th, and data starts right after
            switch (incomingPacket[0])
            {
            case PACKET_HANDSHAKE:
                // Assume handshake successful, don't check it
                // But proper handshake should contain "Hey OVR =D 5" ASCII string right after the packet number
                // Starting on 14th byte (packet number, 12 bytes greetings, null-terminator) we can transfer SlimeVR handshake data
                host = Udp.remoteIP();
                port = Udp.remotePort();
                lastPacketMs = now;
                connected = true;

                Serial.printf("[Handshake] Handshake successful, Slime server is %s:%d\n", Udp.remoteIP().toString().c_str(), +Udp.remotePort());
                // UI::SetMessage(6);
                return;
            default:
                continue;
            }
        }
        else
        {
            break;
        }
    }
    if (lastConnectionAttemptMs + 1000 < now)
    {
        lastConnectionAttemptMs = now;
        Serial.println(F("Looking for the Slime server..."));
        UI::SetMessage(4);
        Network::sendHandshake();
    }
}

uint32_t ServerConnection::getUnixTime(){
    timeClient.update();
    return (uint32_t) timeClient.getEpochTime();
}

void ServerConnection::resetConnection()
{
    Udp.begin(port);
    Haptics_Udp.begin(Haptics_port);
    connected = false;
    Haptics_connected = false;
    timeClient.begin();
}

void ServerConnection::Haptics_update()
{
    if (Haptics_connected)
    {
        if (Haptics_Udp.parsePacket())
        {
            int packetSize = Haptics_Udp.read(incomingPacket, sizeof(incomingPacket));
            if (packetSize)
            {

                // Serial.printf("Received %d bytes from %s, port %d : ", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
                // for (int pointer = 0; pointer < packetSize; pointer++)
                // {
                //     if (pointer > 0)
                //         Serial.print(",");
                //     Serial.print(incomingPacket[pointer], HEX);
                // }
                // Serial.println();

                switch (incomingPacket[0])
                {

                case PACKET_OCTOSLIME_HAPTICS_HEARTBEAT:
                   // Serial.println("Haptics Server Heartbeat received");
                    break;

                case PACKET_OCTOSLIME_HAPTICS_SET:
                    // Octoslime Haptics call
                    Serial.printf("Haptics Setting Device MUX : %d Controller : %d Motor : %d Level : %d",incomingPacket[1], incomingPacket[2], incomingPacket[3], incomingPacket[4]);
                    Haptics::SetLevel(incomingPacket[1], incomingPacket[2], incomingPacket[3], incomingPacket[4]);
                    break;

                case PACKET_OCTOSLIME_HAPTICS_RECEIVE_HANDSHAKE:
                   // Serial.println("Haptics Handshake received again, ignoring");
                    break;

                case PACKET_OCTOSLIME_HAPTICS_RECIEVE_INFO:
                    //Serial.println("Haptics Device Registration Recieved");

                    // Serial.print("Mux ID : ");
                    // Serial.print(incomingPacket[1], HEX);
                    // Serial.print("Controler ID : ");
                    // Serial.print(incomingPacket[2], HEX);
                    // Serial.print("Value : ");
                    // Serial.println(incomingPacket[3], HEX);

                    Haptics::SetRegisteredState(incomingPacket[1], incomingPacket[2], incomingPacket[3]);
                    break;
                }

                LastIncommingHapticMessageTime = millis();
            }
        }

        if (LastIncommingHapticMessageTime + TIMEOUT < millis())
        {
            Haptics_connected = false;
            LastIncommingHapticMessageTime = millis();
          //  Serial.println("Haptic Server Timeout");
        }

        updateOctoSlimeHapticState();
    }
    else
    {
        for (uint8_t MuxID = 0; MuxID < 8; MuxID++)
        {
            for (int ControlerID = 0; ControlerID < 8; ControlerID++)
            {
                Haptics::SetRegisteredState(MuxID, ControlerID, 0);
            }
        }
        Hapticsconnect();
    }
}

bool ServerConnection::IsHapticServerConnected()
{

    return Haptics_connected;
}

bool ServerConnection::IsSlimeServerConnected()
{

    return connected;
}

void ServerConnection::update(Sensor *Sensors[])
{
    if (connected)
    {
        int packetSize = Udp.parsePacket();
        if (packetSize)
        {
            lastPacketMs = millis();
            int len = Udp.read(incomingPacket, sizeof(incomingPacket));
            switch (convert_chars<int>(incomingPacket))
            {
            case PACKET_RECEIVE_HEARTBEAT:
                Network::sendHeartbeat();
                break;
            case PACKET_RECEIVE_VIBRATE:

                break;

            case PACKET_RECEIVE_HANDSHAKE:
                // Assume handshake successful
                Serial.println(F("Handshake received again, ignoring"));
                break;

            case PACKET_RECEIVE_COMMAND:

                break;
            case PACKET_CONFIG:

                break;
            case PACKET_PING_PONG:
                returnLastPacket(len);
                break;

            case PACKET_SENSOR_INFO:
                if (len < 6)
                {
                    Serial.println(F("Wrong sensor info packet"));
                    break;
                }

                sensorStateNotifieds[incomingPacket[4]] = incomingPacket[5];
                //   Serial.print("Incomming sensor packet : ");
                //   Serial.print(incomingPacket[4]);
                //   Serial.print(",");
                //   Serial.println(incomingPacket[5]);
                break;
            }
        }
        // while(Serial.available()) {
        //     size_t bytesRead = Serial.readBytes(serialBuffer, min(Serial.available(), sizeof(serialBuffer)));
        //     sendSerial(serialBuffer, bytesRead, PACKET_SERIAL);
        // }
        if (lastPacketMs + TIMEOUT < millis())
        {
            connected = false;

            for (int BankCount = 0; BankCount < 2; BankCount++)
            {
                for (int SensorCount = 0; SensorCount < IMUCount; SensorCount++)
                {
                    sensorStateNotifieds[SensorCount + (BankCount * IMUCount)] = 0;
                }
            }

            Serial.println(F("Connection to server timed out"));
            UI::SetMessage(5);
        }

        updateSensorState(Sensors);
    }
    else
    {
        Slimeconnect();
    }
}
