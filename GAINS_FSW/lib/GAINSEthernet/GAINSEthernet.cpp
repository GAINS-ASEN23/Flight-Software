/*
 * ETH_init.c
 *
 * Created on: 8 Mar. 2023
 * Author: 
 * 
 * Ethernet Initializer
 */

#include <GAINSEthernet.h>

int ETH_init()
{
// start the Ethernet
  Ethernet.begin(mac1, ip1);

  // Open serial communications and wait for port to open:
  //Serial.begin(9600);
  /*while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }*/

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    //Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    return 1;
    //while (true) {
    //  delay(1); // do nothing, no point running without Ethernet hardware
    //}
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    return 2;
    //Serial.println("Ethernet cable is not connected.");
  }

  // start UDP
  Udp1.begin(localPort1);
  return 0;

}

int UDP_receive()
{
    // if there's data available, read a packet
    int packetSize = Udp1.parsePacket();
    if (packetSize != 0) {
        remote1 = Udp1.remoteIP();

        // read the packet into packetBufffer
        Udp1.read(packetBuffer1, UDP_TX_PACKET_MAX_SIZE);

        /*
        Serial.print("Received packet of size ");
        Serial.println(packetSize);
        Serial.print("From ");
        for (int i=0; i < 4; i++) {
        Serial.print(remote[i], DEC);
            if (i < 3) {
            Serial.print(".");
            }
        }
        Serial.print(", port ");
        Serial.println(Udp.remotePort());
        Serial.println("Contents:");
        Serial.println(packetBuffer);
        */
    }
    return packetSize;


}

void UDP_send()
{

        // send a reply to the IP address and port that sent us the packet we received
        Udp1.beginPacket(Udp1.remoteIP(), Udp1.remotePort());
        Udp1.write(ReplyBuffer1);
        Udp1.endPacket();

}

void SPP_decode()
{



}