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

void setup() {

	pinMode(LED_BUILTIN, OUTPUT);

	// Open serial communications and wait for port to open:
	Serial.begin(115200);
	while (!Serial){
		;
	}

	delay(500);

}

void loop() {

	digitalWrite(LED_BUILTIN, HIGH);

	// Configure connection settings
	int local[] = {21,0,0,103};
	int remote[] = {21,0,0,2};
	int subnet[] = {255,255,255,0};
	int port = 8888;

	// Initialize GAINSEthernet object
	GAINSEthernet GE(local, remote, subnet, port);

	// Print connection settings to serial
	Serial.printf("=======================  Ethernet Configuration  =======================\n");
	GE.printMAC();
	Serial.printf("Local IP: %d:%d:%d:%d\n", local[0], local[1], local[2], local[3]);
	Serial.printf("Remote IP: %d:%d:%d:%d\n", remote[0], remote[1], remote[2], remote[3]);
	Serial.printf("Subnet Mask: %d:%d:%d:%d\n", subnet[0], subnet[1], subnet[2], subnet[3]);
	Serial.printf("Port: %d\n", port);
	Serial.printf("========================================================================\n");


	while (true) {
		GE.read();
		delay(10);
	}

	
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

	digitalWrite(LED_BUILTIN, LOW);
	exit(0);
}