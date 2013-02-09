
#ifndef _INTERPOLATION_H_
#define _INTERPOLATION_H_

#include "types.h"
#include "common.h"
#include "position.h"

#define INTERPOLATION_IGNORE_LEGS    1
#define INTERPOLATION_IGNORE_TURRETS 2
#define INTERPOLATION_IGNORE_GUNS    4

typedef struct {
	POSITION ps; // interpolation starting position
	POSITION pe; // interpolation ending position
	u16 period;
	u16 position;
	s16 delta[(NUM_LEGS+NUM_TURRETS+NUM_GUNS)*3];
	u08 ignore_mask;
} INTERPOLATION;

extern INTERPOLATION interp_legs;
extern INTERPOLATION interp_turrets;
extern INTERPOLATION interp_guns;

u08 interpolation_init(INTERPOLATION *I, POSITION *p1, POSITION *p2, u08 ignore_mask);
u08 interpolation_step(INTERPOLATION *I, POSITION *p, u16 step_size);

#endif