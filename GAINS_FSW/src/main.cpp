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
//#include <GAINSEthernet.h>
#include <NativeEthernetUDP.h>

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(169,254,64,233);//ip(192, 168, 1, 177);
//IPAddress ip2(169,254,102,36);
IPAddress subnet(255,255,0,0);


unsigned int localPort = 8888;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged from Teensy";        // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

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

// start the Ethernet
  Ethernet.begin(mac, ip, subnet);

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

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

  Serial.println("Initialized...");

}


void loop() 
{
	delay(1000);
	Serial.println("Iteration ... The Teensy's IP is: ");
	Serial.println(Ethernet.localIP());
	//Serial.println(Udp.localPort());
	//delay(500);

	// delay(10);
	// int packetSize = UDP_receive();
	// if(packetSize > 0){
	// 	UDP_send();
	// }

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
	
	/*
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