/*
 * ETH_init.c
 *
 * Created on: 8 Mar. 2023
 * Author: 
 * 
 * Ethernet Initializer
 */

#include <GAINSEthernet.h>

void teensyMAC(uint8_t *mac)
{
  for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
  for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
  Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

int ETH_init()
{
// start the Ethernet
  Ethernet.begin(mac, ip);

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
  Udp.begin(localPort);
  return 0;

}

int UDP_receive()
{
    // if there's data available, read a packet
    int packetSize = Udp.parsePacket();
    if (packetSize != 0) {
        remote = Udp.remoteIP();

        // read the packet into packetBufffer
        Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);

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
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(replyBuffer);
        Udp.endPacket();

}

void SPP_decode()
{



}