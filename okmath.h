
#ifndef _OKMATH_H_
#define _OKMATH_H_

#include "types.h"

typedef struct {
	s32 x;
	s32 y;
	s32 z;
} VECTOR;

u16 okmath_vector_magnitude(VECTOR *v);
void okmath_vector_normalize(VECTOR *vi, VECTOR *vo);
void okmath_vector_rotate(VECTOR *vi, VECTOR *vo, s16 roll, s16 pitch, s16 yaw);
u32 okmath_sqrt(u32 num);
s16 okmath_sin(s16 deg);
s16 okmath_cos(s16 deg);
s16 okmath_acos(s16 cosine);
s16 okmath_atan(s16 y, s16 x);

#endif
