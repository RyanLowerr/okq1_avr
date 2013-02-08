
#ifndef _MOTION_H_
#define _MOTION_H_

#include "types.h"
#include "okmath.h"
#include "position.h"
#include "interpolation.h"
#include "controller.h"
#include "gait.h"

// Motion status
#define MOTION_LEG_INTERPOLATING     0
#define MOTION_LEG_WALKING           1
#define MOTION_LEG_TRACKING          2
#define MOTION_LEG_IDLING            3

#define MOTION_TURRET_INTERPOLATING  0
#define MOTION_TURRET_WALKING        1
#define MOTION_TURRET_TRACKING       2
#define MOTION_TURRET_IDLING         3

#define MOTION_GUN_INTERPOLATING     0
#define MOTION_GUN_WALKING           1
#define MOTION_GUN_TRACKING          2
#define MOTION_GUN_IDLING            3

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
