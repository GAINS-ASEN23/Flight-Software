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

//#include <time.h> 						// For srand, clock

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(115200);
}


void loop() 
{
	delay(2000);
	Serial.println("Initialized...");


	for (int i = 0; i<1000; i++)
	{

		digitalWrite(LED_BUILTIN, HIGH); 
		KF();
		digitalWrite(LED_BUILTIN, LOW);


		//delay(1000);

	}

	exit(0);

}