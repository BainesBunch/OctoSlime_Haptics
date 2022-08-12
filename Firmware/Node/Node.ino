
#include <Wire.h>
#include <SoftPWM.h>


uint8_t Base_Address = 0x30;
uint8_t  Registers[9] = { 0xFF,0,0,0,0,0,0,0,0 };
uint8_t  MotorPins[8] = { 15,14,10,9,8,7,6,5 }; 

void setup()
{

	pinMode(11, INPUT_PULLUP);
	pinMode(12, INPUT_PULLUP);
	pinMode(13, INPUT_PULLUP);


	Base_Address +=  (!digitalRead(11) ? 1 : 0);
	Base_Address +=  (!digitalRead(12) ? 2 : 0);
	Base_Address +=  (!digitalRead(13) ? 4 : 0);


	Wire.setClock(80000);
	Wire.begin(Base_Address);

	Wire.onRequest(requestEvent);
	Wire.onReceive(receiveEvent);

	Serial.begin(115200);

	Serial.print("Listening on Address 0x");
	Serial.println(Base_Address, HEX);

	SoftPWMBegin();

}

void receiveEvent(int Count)
{
	byte Packet[2];
	byte Pointer = 0;
	if (Count == 2)
	{
		for (Pointer = 0; Pointer < Count; Pointer++)
		{
			Packet[Pointer] = Wire.read();
		}

		Serial.print("Packet 1 : ");
		Serial.println(Packet[0]);
		Serial.print("  Packet 2 : ");
		Serial.println(Packet[1]);



		Registers[Packet[0]] = Packet[1];
	}
}

void requestEvent()
{
}

void loop()
{
	for (uint8_t Pointer = 0; Pointer < 8; Pointer++)
	{
		SoftPWMSet(MotorPins[Pointer], Registers[Pointer], true);
	}
}