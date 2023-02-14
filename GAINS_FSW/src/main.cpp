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


void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(115200);
}


void loop() 
{
	delay(1000);
	Serial.println("Initialized...");

	delay(500);

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
		Serial.print(duration/pow(10,6));
		Serial.print(" second(s):  ");
		Serial.println(counter);

		digitalWrite(LED_BUILTIN, LOW);

		delay(1000);

	}

	exit(0);

}