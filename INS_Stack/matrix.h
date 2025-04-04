#ifndef _matrix_h
#define _matrix_h

void mat_mul3_9(const float *A,const float *B, float *C);
void mat_mul9_3(const float *A,const float *B, float *C);
void mat_reset(void **d, int size);
void mat_fill(float *m, int size);
void mat_copy(float *A, float *B,int size);
#endif
