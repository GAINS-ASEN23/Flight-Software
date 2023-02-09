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
#include <chrono>
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
	delay(1000);


	for (int i = 0; i<1000; i++)
	{

		digitalWrite(LED_BUILTIN, HIGH); 
		
		auto begin = std::chrono::high_resolution_clock::now();

		KF();

		auto end = std::chrono::high_resolution_clock::now();
    	auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - begin);
    
		digitalWrite(LED_BUILTIN, LOW);

		Serial.println(elapsed.count());

		delay(1000);

	}

	exit(0);

}