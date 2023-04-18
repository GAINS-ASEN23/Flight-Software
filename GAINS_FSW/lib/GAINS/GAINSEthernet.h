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
#include <ccsds.h>
#define PACKET_MAX_SIZE 256 // Default is 24 in NativeEthernet library - "UDP_TX_PACKET_MAX_SIZE"


class GAINSEthernet{
    private:

        byte *mac = nullptr;
        IPAddress* localIP = nullptr;
        IPAddress* remoteIP = nullptr;
        IPAddress* subnetIP = nullptr;
        unsigned int localPort;
        unsigned int remotePort;
        uint8_t *packetBuffer = nullptr;
        EthernetUDP UDP;

        headerData packet_header;
        GAINS_TLM_PACKET tlm_packet;
        GAINS_STAR_PACKET star_packet;

        // Assign the MAC address of this specific Teensy
        void teensyMAC(){
            // GAINSEthernet::mac = (byte*)malloc(6 * sizeof(byte));
            for(uint8_t by=0; by<2; by++) GAINSEthernet::mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
            for(uint8_t by=0; by<4; by++) GAINSEthernet::mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
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
            GAINSEthernet::mac = (byte*) malloc (6 * sizeof(byte));
            GAINSEthernet::packetBuffer = (uint8_t*) malloc (PACKET_MAX_SIZE * sizeof(uint8_t));
            teensyMAC();
            ethInit(); 
        }

        // ~GAINSEthernet(){
        //     delete[] GAINSEthernet::mac;
        // }

        // Print ethernet configuration information
        void info(){
            Serial.printf("=======================  Ethernet Configuration  =======================\n");
            Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", GAINSEthernet::mac[0], GAINSEthernet::mac[1], GAINSEthernet::mac[2], GAINSEthernet::mac[3], GAINSEthernet::mac[4], GAINSEthernet::mac[5]);
            Serial.printf("Local: %d.%d.%d.%d:%d\n", (*GAINSEthernet::localIP)[0], (*GAINSEthernet::localIP)[1], (*GAINSEthernet::localIP)[2], (*GAINSEthernet::localIP)[3], GAINSEthernet::localPort);
            Serial.printf("Remote: %d.%d.%d.%d:%d\n", (*GAINSEthernet::remoteIP)[0], (*GAINSEthernet::remoteIP)[1], (*GAINSEthernet::remoteIP)[2], (*GAINSEthernet::remoteIP)[3], GAINSEthernet::remotePort);
            Serial.printf("Subnet Mask: %d.%d.%d.%d\n", (*GAINSEthernet::subnetIP)[0], (*GAINSEthernet::subnetIP)[1], (*GAINSEthernet::subnetIP)[2], (*GAINSEthernet::subnetIP)[3]);
        }

        // Read a CCSDS packet from UDP
        void read(){
            int packetSize = GAINSEthernet::UDP.parsePacket();
            if (packetSize == 4){
                // GS time recieved

            }
            else if (packetSize) {
                IPAddress remote = GAINSEthernet::UDP.remoteIP();
                uint16_t remotePort = GAINSEthernet::UDP.remotePort();
                UDP.read(GAINSEthernet::packetBuffer, packetSize);

                // Decode the CCSDS packet 
                GAINSEthernet::tlm_packet = read_TLM_Packet(GAINSEthernet::packetBuffer);
                packet_header = readHeader(GAINSEthernet::tlm_packet.FullHeader.SpacePacket.Hdr);

                // Serial.printf("Packet from %d.%d.%d.%d:%d of size %d:  ", remote[0], remote[1], remote[2], remote[3], remotePort, packetSize);
                // char received_message[packetSize];
                // for(int i = 0; i<packetSize; i++){
                //     received_message[i] = GAINSEthernet::packetBuffer[i];
                // }
                // Serial.println(received_message);



            }
        }

        // Send a message to a specific IP and port
        void send(uint8_t* replyBuffer, size_t size, IPAddress IP, uint16_t PORT){
            GAINSEthernet::UDP.beginPacket(IP, PORT);
            GAINSEthernet::UDP.write(replyBuffer, size);
            if (GAINSEthernet::UDP.endPacket() == 0) {
                Serial.println("Error Sending Message");
            }
            else {
                //Serial.printf("Packet to %d.%d.%d.%d:%d\n", IP[0],IP[1],IP[2],IP[3],PORT);
            }
        }

        // Send a message to whatever device just sent you a packet
        void send_return(uint8_t* replyBuffer){
            send(replyBuffer, sizeof(replyBuffer), GAINSEthernet::UDP.remoteIP(), GAINSEthernet::UDP.remotePort());
        }

        /*  Getters and Setters  */

        IPAddress getRemoteIP(){ return *GAINSEthernet::remoteIP; }
        int getRemotePort(){ return GAINSEthernet::remotePort; }

        // Function to get the decoded values for use in the KF
        void get_ground_update(float* ground_vector)
        {
            read();

            uint8_t mode = GAINSEthernet::tlm_packet.FullHeader.Sec.Mode;
            float *ret;
            ret[0] = GAINSEthernet::tlm_packet.FullHeader.Sec.Time;
            ret[1] = (float) GAINSEthernet::tlm_packet.position_x;
            ret[2] = (float) GAINSEthernet::tlm_packet.position_y;
            ret[3] = (float) GAINSEthernet::tlm_packet.position_z;
            ret[4] = (float) GAINSEthernet::tlm_packet.velocity_x;
            ret[5] = (float) GAINSEthernet::tlm_packet.velocity_y;
            ret[6] = (float) GAINSEthernet::tlm_packet.velocity_z;
        }
       
        // Function to set the return data vector for the ground
        void send_ground_update(float t1, float* state, uint32_t systime, IPAddress IP, uint16_t port)
        {
            //float time = systime / 1000.0;
            //Serial.printf("GE - time: %.4f\n",time);

            GAINS_TLM_PACKET tlm_send =  GAINS_TLM_PACKET_constructor(state[0], state[1], state[2], state[3], state[4], state[5], t1, 0, 1, 0, 0, 0, 0);
            size_t packet_size = sizeof(tlm_send);
            uint8_t buffer[packet_size];
            memcpy(&buffer, &tlm_send, packet_size);

            send(buffer, packet_size, IP, port);
        }
};

#endif