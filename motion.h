
#ifndef _MOTION_H_
#define _MOTION_H_

#include "types.h"
#include "okmath.h"
#include "position.h"
#include "interpolation.h"
#include "controller.h"
#include "gait.h"

// Motion status
#define MOTION_LEGS_INTERPOLATING     0
#define MOTION_LEGS_WALKING           1
#define MOTION_LEGS_TRACKING          2
#define MOTION_LEGS_IDLING            3

#define MOTION_TURRETS_INTERPOLATING  0
#define MOTION_TURRETS_FOLLOWING      1
#define MOTION_TURRETS_TRACKING       2
#define MOTION_TURRETS_IDLING         3

#define MOTION_GUNS_INTERPOLATING     0
#define MOTION_GUNS_FOLLOWING         1
#define MOTION_GUNS_TRACKING          2
#define MOTION_GUNS_IDLING            3

typedef struct {
	VECTOR heading; // Direction the turret is pointed.
	VECTOR bearing; // Direction and magnitude the robot is to walk.
	
	u08 leg_state;
	u08 turret_state;
	u08 gun_state;
	
	u16 leg_idle_count;
	u16 turret_idle_count;
	u16 gun_idle_count;
} MOTION;

extern MOTION motion;

void motion_init(MOTION *m);
void motion_process(MOTION *m);

#endif
