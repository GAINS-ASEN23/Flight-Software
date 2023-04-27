#include <GAINSfunctions.h>

/*
 * C = A + B
 * len = length of both arrays/'matricies'
 */
void add(float A[], float B[], float C[], uint16_t len)
{
	for (uint16_t i = 0; i < len; i++) {
        C[i] = A[i] + B[i];
    }
}
