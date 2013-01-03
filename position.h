
#ifndef _POSITION_H_
#define _POSITION_H_

#include <avr/io.h>

#include "common.h"
#include "okmath.h"


typedef struct {
	VECTOR foot[NUM_LEGS];
	VECTOR turret[NUM_TURRETS];
	VECTOR gun[NUM_GUNS];
	uint8_t legignore;
	uint8_t turretignore;
	uint8_t gunignore;
} POSITION;

extern POSITION current;
extern POSITION goal;
extern POSITION coxaoffset;
extern POSITION neutral;

typedef struct {
	POSITION ps; // interpolation starting position
	POSITION pe; // interpolation ending position
	float stepsize[(NUM_LEGS+NUM_TURRETS+NUM_GUNS)*3];
	uint16_t period;
	uint16_t position;
} INTERPOLATION;

void position_init();
void position_copy(POSITION *pi, POSITION *po);
void position_set_goal(POSITION *P);
void position_set_standing(POSITION *p);
void position_set_sitting(POSITION *p);
void position_set_neutral_turrets(POSITION *p);
void position_set_neutral_guns(POSITION *p);
void position_set_neutral(POSITION *p);
uint8_t interpolation_init(INTERPOLATION *I, POSITION *p1, POSITION *p2, uint16_t period);
uint8_t interpolation_step(INTERPOLATION *i, POSITION *p);

#endif
