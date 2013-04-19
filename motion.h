
#ifndef _MOTIONSTATUS_H_
#define _MOTIONSTATUS_H_

#include "types.h"
#include "okmath.h"
#include "position.h"
#include "interpolation.h"
#include "controller.h"
#include "gait.h"

// Motion states
#define MOTIONSTATE_LEGS_INIT        0
#define MOTIONSTATE_LEGS_STAND       1
#define MOTIONSTATE_LEGS_SIT         2
#define MOTIONSTATE_LEGS_WALK        4
#define MOTIONSTATE_LEGS_TRACK       8

#define MOTIONSTATE_TURRET_INIT      0
#define MOTIONSTATE_TURRET_HOME      1
#define MOTIONSTATE_TURRET_ABSOLUTE  2
#define MOTIONSTATE_TURRET_RELATIVE  4
#define MOTIONSTATE_TURRET_TRACK     8

// Motion status
#define MOTIONSTATUS_LEGS_INTERPOLATING     0
#define MOTIONSTATUS_LEGS_WALKING           1
#define MOTIONSTATUS_LEGS_TRACKING          2
#define MOTIONSTATUS_LEGS_IDLING            3

#define MOTIONSTATUS_TURRET_INTERPOLATING   0
#define MOTIONSTATUS_TURRET_FOLLOWING       1
#define MOTIONSTATUS_TURRET_TRACKING        2
#define MOTIONSTATUS_TURRET_IDLING          3

typedef struct {
	// Motion Paramaters for walking, looking with the turret, and aiming the guns.
	s16 travel_x; // Translational travel along the X axis in mm. DEC1
	s16 travel_y; // Translational travel along the Y axis in mm. DEC1
	s16 travel_r; // Rotational travel around the Z axis in degrees. DEC1
	s16 travel_l; // Leg lift height in mm. DEC1
	s16 travel_s; // Speed of travel.
	u08 travel_request;
	u08 travel_largechange;
	
	u08 state_leg; 
	u08 status_leg;
	
	// Idle counters.
	u16 idle_count;
} MOTION;

extern MOTION motion;

void motion_init(MOTION *m);
void motion_process(MOTION *m, CONTROLLER *c);

#endif