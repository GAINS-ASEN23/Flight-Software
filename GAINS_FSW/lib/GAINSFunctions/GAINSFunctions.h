/*
 * GAINSFunctions.h
 *
 *  Created on: 7 Feb. 2023
 *      Author: Bennett Grow
 */

#ifndef _GAINSFunctions_H_
#define _GAINSFunctions_H_

#include <stdlib.h>						// Standard library
#include <stdint.h>						// For uint8_t, uint16_t and uint16_t



void copy(float A[], float B[], uint16_t len);
void add(float A[], float B[], float C[], uint16_t len);
void sub(float A[], float B[], float C[], uint16_t len);
void eye(float A[], uint16_t N);

#endif