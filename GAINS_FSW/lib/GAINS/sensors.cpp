/*
============================================================================
Name        : sensors.cpp
Author      : Bennett Grow
Version     : 0.1
Copyright   : 
Description : 
============================================================================
*/

#include <sensors.h>

// Convert accelerometer differential pin inputs to g's (no corrections)
float sensors::AO_to_g(int AP, int AN){
    float bins = 4096;         // Bins in analog output
    float range = 2.0;         // +/- 2 g output
    float diff = AP - AN;  // Differential bin output

    float accel = (diff * range) / bins;
    return correct_accel(accel);
}

// Convert temperature pin inputs to degrees C
float sensors::temp(float VT){
    return ((VT*90.0)/4096.0) + 35.0;  // Currently doesn't work
}

// Convert g's to m/s^2 based on local testing altitude
float sensors::local_g_to_ms2(float g){
    
}

// Correct acceleration bias and scale factor
float sensors::correct_accel(float accel){


}

