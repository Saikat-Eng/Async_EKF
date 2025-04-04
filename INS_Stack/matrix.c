#include "matrix.h"
#include <math.h>
#include <stdio.h>

//size in byte
void mat_reset(void **d, int size)
{
    int rem = size%8;
    size -= rem;

    while(size)
    {
        *((float**)d++) = 0;
        size -= 8;
    }

    if(rem>0)
        *((float*)d) = 0;
}

void mat_fill(float *B, int size)
{
    while(size--)
    {
        *B++ = 0.0f;
    }
}

void mat_copy(float *A, float *B,int size)
{
    int rem = size % 28;
    if(rem==0)
    {
        for(rem=0;rem<size;)
        {
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
            B[rem] = A[rem]; rem++;
        }
    }
    else
    {
        while(size--)
        {
            *B++ = *A++;
        }
    }
}

void mat_mul9_3(const float *A,const float *B, float *C)
{
    C[0] = A[0]*B[0]+A[1]*B[1]+A[2]*B[2];
    C[1] = A[3]*B[0]+A[4]*B[1]+A[5]*B[2];
    C[2] = A[6]*B[0]+A[7]*B[1]+A[8]*B[2];
}

void mat_mul3_9(const float *A,const float *B, float *C)
{
    C[0] = A[0]*B[0]+A[1]*B[3]+A[2]*B[6];
    C[1] = A[0]*B[1]+A[1]*B[4]+A[2]*B[7];
    C[2] = A[0]*B[2]+A[1]*B[5]+A[2]*B[8];
}

