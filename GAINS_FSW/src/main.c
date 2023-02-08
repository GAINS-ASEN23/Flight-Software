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

}


void loop() 
{



	for (int i = 0; i<1000; i++)
	{
		//BEGINNING = micros();
		digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
		KF();
		digitalWrite(LED_BUILTIN, LOW);


		delay(1000);

	}

	exit(0);
}
