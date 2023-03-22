/*
 * GAINSEthernet.h
 *
 * Created on: 1 Mar. 2023
 * Author: Bennett Grow
 */

#ifndef _GAINSEthernet_H_
#define _GAINSEthernet_H_

//#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

//#ifdef __cplusplus
//extern "C" {
//#endif

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
//#ifndef _GAINS_IP_Stuff_
//#define _GAINS_IP_Stuff_

byte mac1[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip1(192, 168, 1, 177);

unsigned int localPort1 = 8888;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer1[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
char ReplyBuffer1[] = "acknowledged";        // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp1;

IPAddress remote1;

//#endif


int ETH_init();
void SPP_decode();
void UDP_send();
int UDP_receive();


//#ifdef __cplusplus
//}
//#endif

#endif