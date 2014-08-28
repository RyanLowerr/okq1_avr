
#ifndef _INTERPOLATION_H_
#define _INTERPOLATION_H_

#include "types.h"
#include "common.h"
#include "position.h"

typedef struct {
	POSITION ps; // interpolation starting position
	POSITION pe; // interpolation ending position
	u16 period;
	u16 position;
	s16 delta[(NUM_LEGS+1)*3];
} INTERPOLATION;

extern INTERPOLATION interp;

u08 interpolation_init(INTERPOLATION *I, POSITION *p1, POSITION *p2);
u08 interpolation_step(INTERPOLATION *I, POSITION *p, u16 step_size);

#endif
