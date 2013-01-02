
#ifndef _MOTION_H_
#define _MOTION_H_

#include <avr/io.h>

#include "vector.h"
#include "position.h"
#include "controller.h"
#include "gait.h"

typedef struct {
	VECTOR heading; // Direction the turret is pointed.
	VECTOR bearing; // Direction and magnitude the robot is to walk.
} MOTION;

void motion_init(void);
void motion_capture_position(void);
void motion_process(CONTROLLER *controller, GAIT *gait);

#endif
