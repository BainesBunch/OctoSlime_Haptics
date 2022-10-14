#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "LPDR_Home_Automation";
const char* password = "50058556";

WiFiUDP Udp;
unsigned int localUdpPort = 2022;  // local port to listen on
char incomingPacket[255];  // buffer for incoming packets
char  replyPacket[] = "Hi there! Got the message :-)";  // a reply string to send back


void setup()
{
  Serial.begin(115200);
  Serial.println();

  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

  Udp.begin(localUdpPort);

  Udp.beginPacket(IPAddress(255, 255, 255, 255), localUdpPort);

        Udp.write('A');
        Udp.write('B');
        Udp.write('C');
Udp.endPacket();
  
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

}


void loop()
{
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    // receive incoming UDP packets
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }

    for (int i = 0; i < len; ++i) Serial.print((char)incomingPacket[i]);
     Serial.println();
    

    // send back a reply, to the IP address and port we got the packet from
  
     Serial.print("Sending reply : ");
     Serial.println(replyPacket);

    
    
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(replyPacket);
    Udp.endPacket();

  
  
  }
}
