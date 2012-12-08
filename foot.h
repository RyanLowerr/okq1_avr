
#ifndef _FOOT_H_
#define _FOOT_H_

#include <avr/io.h>

#include "controller.h"
#include "gait.h"

typedef struct
{
	float x;
	float y;
	float z;
	float neutral_from_coxa_x;
	float neutral_from_coxa_y;
	float neutral_from_center_x;
	float neutral_from_center_y;
	float neutral_z;
	float coxa_offset_x;
	float coxa_offset_y;
} footdata;

void foot_init(footdata* foot);
void foot_position_calc(footdata* foot, controllerdata* controller, gaitdata* gait);

#endif
