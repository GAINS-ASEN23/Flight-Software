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

//#define HWSERIAL Serial1

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(115200);
	//HWSERIAL.begin(115200);
}


void loop() 
{
	delay(2000);
	Serial.println("Initialized...");
	//HWSERIAL.println("Initialized...");

	delay(1000);


	for (int i = 0; i<1000; i++)
	{
		digitalWrite(LED_BUILTIN, HIGH); 
		unsigned long start = micros();

		for (int i = 0; i<100; i++)
		{	


			KF();

		}

		unsigned long end = micros();
		unsigned long duration = end - start; 

		Serial.print(i);
		Serial.print("  Time to execute 100 KF cycles:  ");
		Serial.print(duration);
		Serial.print(" microseconds / ");
		Serial.print(duration/pow(10,3));
		Serial.println(" milliseconds ");

		//HWSERIAL.println(i);
		digitalWrite(LED_BUILTIN, LOW);
		delay(1000);
	}
	

	exit(0);

}