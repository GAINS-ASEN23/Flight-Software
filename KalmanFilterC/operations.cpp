/*
============================================================================
Name        : operations.h
Author      : Bennett Grow, Kaylie Rick, Jason Popich
Version     : 0.1
Copyright   : 
Description : This file contains some mathematical operation function definitions 
               that could be needed in the main KF source code
============================================================================
*/

#include "operations.hpp"

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

/*
 * C = A - B
 * len = length of both arrays/'matricies'
 */
void sub(float A[], float B[], float C[], uint16_t len) 
{
	for (uint16_t i = 0; i < len; i++) {
        C[i] = A[i] - B[i];
    }
}

/*
 * A = Identity Matrix
 * N = side length
 */
void eye(float A[], uint16_t N)
{
    for(int i = 0; i < N; i++)
    {
        for(int j = 0; j < N; j++)
        {
            if(i == j)
            {
                A[i*N + j] = 1;
            }
            else
            {
                A[i*N + j] = 0;
            }
        }
    }
}