/*
 ============================================================================
 Name        : main.cpp
 Author      : Bennett Grow, Kaylie Rick, Jason Popich
 Version     : 0.2
 Copyright   : 
 Description : Main script to run all GAINS Flight Software
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
	Serial.printf("https://github.com/GAINS-ASEN23\n");

	digitalWrite(LED_BUILTIN, HIGH);

	// Configure connection settings
	int local[] = {21,0,0,103};
	int localport = 8888;
	int remote[] = {21,0,0,3};
	int remoteport = 8889;
	int subnet[] = {255,255,255,0};

	// Initialize GAINSEthernet object and print connection information
	GAINSEthernet GE(local, remote, subnet, localport, remoteport);
	GE.info();
	Serial.printf("========================================================================\n");

	// Recieve and send a message over UDP
	char message[] = {"Teensy send test"};
	while (true) {
		Serial.printf("Loop Start");
		delay(5000);
		GE.read();
		GE.send(message, GE.getRemoteIP(), GE.getRemotePort());
		// GE.send(message, IPAddress(21,0,0,3), 8889);  // Alternative usage
	}


	// At each timestep 
	// 		Read in sensor data
	// 		Read any packets/GS updates
	//		Coordinate frame trans/etc.
	// 		run 1 step of KF
	//		send state to ground or write to memory if LOC



	digitalWrite(LED_BUILTIN, LOW);
	exit(0);
}