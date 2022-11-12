#include <stdlib.h>						// Standard library
#include <stdint.h>						// For uint8_t, uint16_t and uint16_t

/*
 * C = A + B
 * len = length of both arrays/'matricies'
 */
void add(float A[], float B[], float C[], uint16_t len) {

	for (uint16_t i = 0; i < len; i++) {
        C[i] = A[i] + B[i];
    }

}