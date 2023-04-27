#include <GAINSfunctions.h>

/*
 * B = A
 * len = length of both arrays/'matricies'
 */
void copy(float A[], float B[], uint16_t len)
{
	for (uint16_t i = 0; i < len; i++) {
        B[i] = A[i];
    }
}