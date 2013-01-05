
#ifndef _OKMATH_H_
#define _OKMATH_H_

#include "types.h"

typedef struct {
	float x;
	float y;
	float z;
} VECTOR;

void vector_normalize(VECTOR *vi, VECTOR *vo);
void vector_magnitude(VECTOR *v);
void vector_rotate(VECTOR *vi, VECTOR *vo, float roll, float pitch, float yaw);

s16 lookupsin(s16 deg);
s16 loopupcos(s16 deg);

#endif
