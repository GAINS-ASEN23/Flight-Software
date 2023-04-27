/*
 * GAINSEthernet.h
 *
 * Created on: 1 Mar. 2023
 * Author: Bennett Grow
 */

#ifndef _GAINSEthernet_H_
#define _GAINSEthernet_H_


// #include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
// #define UDP_TX_PACKET_MAX_SIZE = 24; // Default is 24 in NativeEthernet library

#ifdef __cplusplus
extern "C" {
#endif


byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

IPAddress ip(192, 168, 1, 177);

unsigned int localPort = 8888;              // local port to listen on

char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
char replyBuffer[] = "acknowledged";        // a string to send back

EthernetUDP Udp;
IPAddress remote;


void teensyMAC(uint8_t *mac);
int ETH_init();
void SPP_decode();
void UDP_send();
int UDP_receive();
void SPP_decode();

#ifdef __cplusplus
}
#endif

#endif