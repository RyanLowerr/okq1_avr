
#ifndef _LEG4DOF_H_
#define _LEG4DOF_H_

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

u08 leg4dof_kinematics_reverse(s16 x, s16 y, s16 z, s16 *coxa, s16 *femur, s16 *tibia, s16 *tarsus);
u08 leg4dof_kinematics_forward(void);
u08 leg4dof_interpolation_init(INTERPOLATION *I, POSITION *p1, POSITION *p2);
u08 leg4dof_interpolation_step(INTERPOLATION *I, POSITION *p, u16 step_size);

#endif
