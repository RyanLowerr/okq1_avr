
#ifndef _MOTIONMANADER_H_
#define _MOTIONMANAGER_H_

#include "types.h"
#include "okmath.h"
#include "position.h"
#include "interpolation.h"
#include "controller.h"
#include "gait.h"

#define MOTION_POWER_UP            0
#define MOTION_INITALIZE           1
#define MOTION_INTERPOLATE         2
#define MOTION_INTERPOLATE_DONE    3
#define MOTION_GAIT                4
#define MOTION_LEGS_DOWN           5
#define MOTION_LEGS_UP             6
#define MOTION_LEGS_NEUTRAL        7

typedef struct {
	VECTOR heading; // Direction the turret is pointed.
	VECTOR bearing; // Direction and magnitude the robot is to walk.
	u08 state;
} MOTIONMANAGER;

extern MOTIONMANAGER motionmanager;

void mm_init(void);
void mm_process(void);

#endif
