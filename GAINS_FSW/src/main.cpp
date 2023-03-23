/*
 ============================================================================
 Name        : main.c
 Author      : Bennett Grow, Kaylie Rick, Jason Popich
 Version     : 0.1
 Copyright   : 
 Description : 
 ============================================================================
 */

#include <Arduino.h>
#include <GAINSKF.h>
#include <GAINSEthernet.h>
//#include <NativeEthernetUDP.h>
/*
u_int8_t mac[] = {
  //0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
  
  // Teensy 3
  // 04:e9:e5:14:31:44
  // 10.0.0.103 on BG's home network
  0x04, 0xE9, 0xE5, 0x14, 0x31, 0x44
};

// Specs for the Teensy
IPAddress ip(10,0,0,103);
IPAddress subnet(255,255,255,0);
unsigned int localPort = 8888;

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  			// buffer to hold incoming packet,
char ReplyBuffer[] = "Acknowledged from Teensy";        // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
*/

void setup()
{

	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(115200);

	delay(1000);
	Serial.println("Initialized...");

	delay(500);

	/*
	int eth_err_code = ETH_init();
	if(1 == eth_err_code){
		Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
	} else if(2 == eth_err_code){
		Serial.println("Ethernet cable is not connected.");
	}
	*/

	// Open serial communications and wait for port to open:
	Serial.begin(9600);
	while (!Serial) {
	; // wait for serial port to connect. Needed for native USB port only
	}
	/*
	// Find the teensy's actual mac address
	for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
	for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
	Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	// Serial.printf("IP|PORT: %i.%i.%i.%i|%i\n", ip[1], localPort);
	// Serial.printf("SUBNET: %i.%i.%i.%i\n", ip, localPort);



	// Start ethernet using the mac, ip, and subnet mask OF THE TEENSY
	Ethernet.begin(mac, ip, subnet);

	// Check for Ethernet hardware present
	if (Ethernet.hardwareStatus() == EthernetNoHardware) {
		Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
		while (true) {
			delay(1); // do nothing, no point running without Ethernet hardware
		}
	}
	if (Ethernet.linkStatus() == LinkOFF) {
		Serial.println("Ethernet cable is not connected.");
	}

	// start UDP
	Udp.begin(localPort);
	*/

}


void loop() 
{
	delay(1000);

	

	// teensyMAC(mac);
	// ETH_init();

	/*
	Serial.println("Iteration ... The Teensy's IP is: ");
	Serial.println(Ethernet.localIP());


	// if there's data available, read a packet
  	int packetSize = Udp.parsePacket();
  	if (packetSize) {
		Serial.print("Received packet of size ");
		Serial.println(packetSize);
		Serial.print("From ");
		IPAddress remote = Udp.remoteIP();
		for (int i=0; i < 4; i++) {
			Serial.print(remote[i], DEC);
			if (i < 3) {
				Serial.print(".");
			}
		}
		Serial.print(", port ");
		Serial.println(Udp.remotePort());

		// read the packet into packetBufffer
		Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
		Serial.println("Contents:");
		Serial.println(packetBuffer);

		
		// send a reply to the IP address and port that sent us the packet we received
		Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
		Udp.write(ReplyBuffer);
		Serial.println("Sent Message:");
		Serial.println(ReplyBuffer);
		int send_error = Udp.endPacket();
			if (send_error == 0){
				Serial.println("Error Sending Message");
			}
  	}

  	delay(10);
	

	for (size_t i = 0; i < 1000; i++)
	{
		
		digitalWrite(LED_BUILTIN, HIGH); 

		unsigned long start = micros();
		unsigned long now;
		unsigned long duration;
		int counter = 0;

		while (true)
		{

			KF();

			now = micros();
			duration = now - start; 

			counter += 1;

			if (duration >= 1000000)
			{
				break;
			}

		}

		Serial.print("KF cycles in ");
		Serial.print(duration/pow(10,3));
		Serial.print(" milliseconds:  ");
		Serial.println(counter);

		digitalWrite(LED_BUILTIN, LOW);

		delay(1000);

	}
	*/

	//exit(0);
	digitalWrite(LED_BUILTIN, LOW);
}

