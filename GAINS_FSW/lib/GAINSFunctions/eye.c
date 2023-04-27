#include <GAINSfunctions.h>

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