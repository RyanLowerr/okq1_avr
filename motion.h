
#ifndef _MOTIONSTATUS_H_
#define _MOTIONSTATUS_H_

#include "types.h"
#include "controller.h"

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

void motion_init(void);
void motion_process(CONTROLLER *c);

#endif
