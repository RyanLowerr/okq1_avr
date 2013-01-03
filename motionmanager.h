
#ifndef _MOTIONMANADER_H_
#define _MOTIONMANAGER_H_

#include "types.h"
#include "okmath.h"
#include "position.h"
#include "controller.h"
#include "gait.h"

typedef struct {
	VECTOR heading; // Direction the turret is pointed.
	VECTOR bearing; // Direction and magnitude the robot is to walk.
} MOTIONMANAGER;

void mm_init(void);
void mm_process(CONTROLLER *controller, GAIT *gait);

#endif
