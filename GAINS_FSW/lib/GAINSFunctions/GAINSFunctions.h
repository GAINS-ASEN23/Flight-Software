/*
 ============================================================================
 Name        : GAINSFunctions.h
 Author      : Bennett Grow
 Version     : 0.1
 Copyright   : 
 Description : Various functions useful for matrix math and other GAINS functions
 ============================================================================
 */

#ifndef _GAINSFunctions_H_
#define _GAINSFunctions_H_

#include <stdlib.h>						// Standard library
#include <stdint.h>						// For uint8_t, uint16_t and uint16_t

#ifdef __cplusplus
extern "C" {
#endif

// Matrix math
void copy(float A[], float B[], uint16_t len);
void add(float A[], float B[], float C[], uint16_t len);
void sub(float A[], float B[], float C[], uint16_t len);
void eye(float A[], uint16_t N);

#ifdef __cplusplus
}
#endif

#endif