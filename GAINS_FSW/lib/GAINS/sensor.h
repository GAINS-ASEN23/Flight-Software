/*
============================================================================
Name        : sensor.h
Author      : Bennett Grow
Version     : 0.1
Copyright   : 
Description : Sensor processing
============================================================================
*/

#ifndef _sensor_H
#define _sensor_H

#include <Arduino.h>
#include <stdlib.h>				// Standard library
#include <stdint.h>			    // For uint8_t, uint16_t and uint16_t


#define ADC_RES 12 			    // [bits] ADC resolution
#define AP 24 				    // Positive accelerometer differential pin
#define AN 25 				    // Negative accelerometer differential pin
#define VT 20 				    // Temperature pin from accelerometer

class sensor{
    private:
        const float G_SL = 9.80665;                     // [m/s^2] Gravitational acceleration at sea level
        const float EARTH_RAD = 6378100.0;              // [m] Earth's average radius at sea level

        float local_altitude = 1606;                    // [m] Local testing altitude from Earth sea level
        float scale_factor = 1.135517722;                       // Accelerometer bias
        float bias = -0.006750653;              // Accelerometer scale factor

        float A_P = 0;
		float A_N = 0;
		float V_T = 0;

        float get_g();        
        float g_to_ms2(float g);
        float correct_accel(float accel); 



    public:
        float get_accel();

        sensor(){
            analogReadResolution(ADC_RES);
        }
        sensor(float altitude, float accel_bias, float accel_scale_factor){
            analogReadResolution(ADC_RES);
            local_altitude = altitude;
            bias = accel_bias;
            scale_factor = accel_scale_factor;
        };

        float get_acceleration();
        float get_temperature();

};

/*  PUBLIC  */

// Returns current acceleration in m/s^2
float sensor::get_acceleration(){
    return g_to_ms2( correct_accel( get_g() ) );
}

float sensor::get_temperature(){
    int T = analogRead(VT);
    //float corrected = T / 1874.08819409718 - 17.8484022164697;
    return T;
}

/*  PRIVATE  */

// Convert accelerometer differential pin inputs to g's (no corrections)
float sensor::get_g(){
    A_P = analogRead(AP);
    A_N = analogRead(AN);

    float bins = 4096;          // Bins in analog output
    float range = 2.0;          // +/- 2 g output
    float diff = A_P - A_N;     // Differential bin output

    float accel = (diff * range) / bins;
    return accel;
}

// Correct acceleration [g's] bias and scale factor
float sensor::correct_accel(float accel){
    return (accel / scale_factor) - bias;
}

// Convert g's to m/s^2 based on local testing altitude
float sensor::g_to_ms2(float g){
    float intermed = (EARTH_RAD) / (EARTH_RAD + local_altitude);
    float ref_g = G_SL * (intermed * intermed);
    return ref_g * g;
}



#endif