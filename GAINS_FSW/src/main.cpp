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

	Serial.printf("===============================  GAINS  ================================\n");
	Serial.printf("General Atomics Inertial Navigation System\n");
	Serial.printf("University of Colorado Boulder\n");
	Serial.printf("Ann and H.J. Smead Dept. of Aerospace Engineering\n");
	Serial.printf("Senior Design Project - Spring 2023\n");


	digitalWrite(LED_BUILTIN, HIGH);

	// Configure connection settings
	int local[] = {21,0,0,103};
	int remote[] = {21,0,0,3};
	int subnet[] = {255,255,255,0};
	int port = 8888;

	// Initialize GAINSEthernet object
	GAINSEthernet GE(local, remote, subnet, port);
	GE.info();
	Serial.printf("========================================================================\n");


	char message[] = {"Teensy send test"};
	//uint16_t send_port = 8889;

	while (true) {
		GE.read();
		delay(500);
		GE.send(message, IPAddress(21,0,0,3), 8889);
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