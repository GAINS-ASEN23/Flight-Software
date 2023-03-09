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


}