/*
 ============================================================================
 Name        : GAINSEthernet.h
 Author      : Bennett Grow
 Version     : 0.1
 Copyright   : 
 Description : Utilizes NativeEthernet to send and receive packets
 ============================================================================

    GAINSEthernet() to set ethernet configuration and initialize connection
    info() to print connection configuration details
    read() to read any incoming packets
    send() to send a packet

 */

#ifndef _GAINSEthernet_H_
#define _GAINSEthernet_H_

#include <NativeEthernetUdp.h>
// #define UDP_TX_PACKET_MAX_SIZE = 24; // Default is 24 in NativeEthernet library


class GAINSEthernet{
    private:

        byte mac[6];
        IPAddress* localIP = nullptr;
        IPAddress* remoteIP = nullptr;
        IPAddress* subnetIP = nullptr;
        unsigned int localPort;
        unsigned int remotePort;
        char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet
        EthernetUDP UDP;

        // Assign the MAC address of this specific Teensy
        void teensyMAC(uint8_t *mac){
            for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
            for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
        }

        // Initialize ethernet and UDP
        void ethInit(){
            Ethernet.begin(GAINSEthernet::mac, *GAINSEthernet::localIP, *GAINSEthernet::subnetIP);
            if (Ethernet.hardwareStatus() == EthernetNoHardware) {
                Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
                while (true) {
                    delay(1); // do nothing, no point running without Ethernet hardware
                }
            }
            if (Ethernet.linkStatus() == LinkOFF) {
                Serial.println("Ethernet cable is not connected.");
            }
            GAINSEthernet::UDP.begin(GAINSEthernet::localPort);
        }

    public:

        // Construct a new GAINSEthernet object using network configuration of the teensy then initialize
        GAINSEthernet(int local[], int remote[], int subnet[], int localport, int remoteport){
            GAINSEthernet::localIP = new IPAddress(local[0], local[1], local[2], local[3]);
            GAINSEthernet::remoteIP = new IPAddress(remote[0], remote[1], remote[2], remote[3]);
            GAINSEthernet::subnetIP = new IPAddress(subnet[0], subnet[1], subnet[2], subnet[3]);
            GAINSEthernet::localPort = localport;
            GAINSEthernet::remotePort = remoteport;
            teensyMAC(mac);
            ethInit(); 
        }

        // Print ethernet configuration information
        void info(){
            Serial.printf("=======================  Ethernet Configuration  =======================\n");
            Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", GAINSEthernet::mac[0], GAINSEthernet::mac[1], GAINSEthernet::mac[2], GAINSEthernet::mac[3], GAINSEthernet::mac[4], GAINSEthernet::mac[5]);
            Serial.printf("Local: %d.%d.%d.%d:%d\n", (*GAINSEthernet::localIP)[0], (*GAINSEthernet::localIP)[1], (*GAINSEthernet::localIP)[2], (*GAINSEthernet::localIP)[3], GAINSEthernet::localPort);
            Serial.printf("Remote: %d.%d.%d.%d:%d\n", (*GAINSEthernet::remoteIP)[0], (*GAINSEthernet::remoteIP)[1], (*GAINSEthernet::remoteIP)[2], (*GAINSEthernet::remoteIP)[3], GAINSEthernet::remotePort);
            Serial.printf("Subnet Mask: %d.%d.%d.%d\n", (*GAINSEthernet::subnetIP)[0], (*GAINSEthernet::subnetIP)[1], (*GAINSEthernet::subnetIP)[2], (*GAINSEthernet::subnetIP)[3]);

            // IPAddress local = *localIP;
            // IPAddress remote = *remoteIP;
            // IPAddress subnet = *subnetIP;
            // Serial.printf("=======================  Ethernet Configuration  =======================\n");
            // Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            // Serial.printf("Local: %d.%d.%d.%d:%d\n", local[0], local[1], local[2], local[3], localPort);
            // Serial.printf("Remote: %d.%d.%d.%d:%d\n", remote[0], remote[1], remote[2], remote[3], remotePort);
            // Serial.printf("Subnet Mask: %d.%d.%d.%d\n", subnet[0], subnet[1], subnet[2], subnet[3]);
            // Serial.printf("========================================================================\n");
        }

        // Read a packet from UDP
        void read(){
            int packetSize = GAINSEthernet::UDP.parsePacket();
            if (packetSize) {
                IPAddress remote = GAINSEthernet::UDP.remoteIP();
                uint16_t remotePort = GAINSEthernet::UDP.remotePort();
                UDP.read(GAINSEthernet::packetBuffer, UDP_TX_PACKET_MAX_SIZE);
                Serial.printf("Packet from %d.%d.%d.%d:%d of size %d:  ", remote[0], remote[1], remote[2], remote[3], remotePort, packetSize);
                Serial.println(GAINSEthernet::packetBuffer);
            }
        }

        // Send a message to a specific IP and port
        void send(char replyBuffer[], IPAddress IP, uint16_t PORT){
            GAINSEthernet::UDP.beginPacket(IP, PORT);
            GAINSEthernet::UDP.write(replyBuffer);
            if (GAINSEthernet::UDP.endPacket() == 0) {
                Serial.println("Error Sending Message");
            }
            else {
                Serial.printf("Packet to %d.%d.%d.%d:%d  >>  %s\n", IP[0],IP[1],IP[2],IP[3],PORT,replyBuffer);
            }
        }

        // Send a message to whatever device just sent you a packet
        void send_return(char replyBuffer[]){
            send(replyBuffer, GAINSEthernet::UDP.remoteIP(), GAINSEthernet::UDP.remotePort());
        }

        /*  Getters and Setters  */

        IPAddress getRemoteIP(){ return *GAINSEthernet::remoteIP; }
        int getRemotePort(){ return GAINSEthernet::remotePort; }

};

#endif