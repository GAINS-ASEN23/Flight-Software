


#ifndef _GAINSEthernet_H_
#define _GAINSEthernet_H_

#include <NativeEthernetUdp.h>
// #define UDP_TX_PACKET_MAX_SIZE = 24; // Default is 24 in NativeEthernet library


class GAINSEthernet{
    public:
        byte mac[6];
        IPAddress* localIP;
        IPAddress* remoteIP;
        IPAddress* subnetIP;
        unsigned int localPort;
        char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
        EthernetUDP Udp;

        // Constructor
        GAINSEthernet(int local[], int remote[], int subnet[], int port){
            localIP = new IPAddress(local[0], local[1], local[2], local[3]);
            remoteIP = new IPAddress(remote[0], remote[1], remote[2], remote[3]);
            subnetIP = new IPAddress(subnet[0], subnet[1], subnet[2], subnet[3]);
            localPort = port;
            teensyMAC(mac);
            ethInit();            
        }

        // Print the MAC address to serial
        void printMAC(){
            Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        }

        // Assign the MAC address of this specific Teensy
        void teensyMAC(uint8_t *mac){
            for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
            for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
        }

        void ethInit(){
            Ethernet.begin(mac, *localIP, *subnetIP);

            if (Ethernet.hardwareStatus() == EthernetNoHardware) {
                Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
                while (true) {
                    delay(1); // do nothing, no point running without Ethernet hardware
                }
            }

            if (Ethernet.linkStatus() == LinkOFF) {
                Serial.println("Ethernet cable is not connected.");
            }

            Udp.begin(localPort);

        }

        void read(){
            int packetSize = Udp.parsePacket();
            if (packetSize) {
                IPAddress remote = Udp.remoteIP();
                uint16_t remotePort = Udp.remotePort();
                Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);

                Serial.printf("Packet from %d.%d.%d.%d:%d of size %d:  ", remote[0], remote[1], remote[2], remote[3], remotePort, packetSize);
                Serial.println(packetBuffer);

                char replyMessage[] = "Message acknowledged from Teensy";
                send(replyMessage);
            }
        }

        void send(char ReplyBuffer[]){
            Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
            Udp.write(ReplyBuffer);

            int send_error = Udp.endPacket();
            if (send_error == 0) {
                Serial.println("Error Sending Message");
            }
            else {
                Serial.printf("Sent Message: %s\n",ReplyBuffer);
            }
        }


};

#endif