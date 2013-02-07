
#ifndef _POSITION_H_
#define _POSITION_H_

#include "types.h"

#include "common.h"
#include "okmath.h"


typedef struct {
	VECTOR foot[NUM_LEGS];
	VECTOR turret[NUM_TURRETS];
	VECTOR gun[NUM_GUNS];
} POSITION;

extern POSITION current;
extern POSITION goal;
extern POSITION coxaoffset;
extern POSITION neutral;

typedef struct {
	POSITION ps; // interpolation starting position
	POSITION pe; // interpolation ending position
	u16 period;
	u16 position;
	s16 delta[(NUM_LEGS+NUM_TURRETS+NUM_GUNS)*3];
	u08 ignore_mask;
} INTERPOLATION;

extern INTERPOLATION interpolation;

void position_init(void);
void position_copy(POSITION *pi, POSITION *po);
void position_set_goal(POSITION *P);
void position_set_standing(POSITION *p);
void position_set_sitting(POSITION *p);
void position_set_neutral_turrets(POSITION *p);
void position_set_neutral_guns(POSITION *p);
void position_set_neutral(POSITION *p);
u08 interpolation_init(INTERPOLATION *I, POSITION *p1, POSITION *p2, u08 ignore_mask);
u08 interpolation_step(INTERPOLATION *I, POSITION *p, u08 step_size);

#endif
