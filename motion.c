
#include <avr/io.h>
#include <math.h>

#include "motion.h"
#include "common.h"
#include "position.h"
#include "kinematics.h"
#include "joint.h"
#include "vector.h"
#include "gait.h"
#include "dynamixel.h"
#include "mx.h"
#include "ax.h"

void motion_init(void)
{
	;
}

void motion_capture_start_position(void)
{
	// read all curret servo positions
	// perform FK to determine inital limb positions
	// setup interpolation between measured inital position and neutral position
	// set "interpolation to be done" flag
	;
}

void motion_process(CONTROLLER *controller, GAIT *gait)
{
	
	if(controller_buffer_size() > 16)
		controller_read(controller);
	
	if(gait->position == gait->period)
	{
		controller->y = (((float) controller->a[0]) - 512.0) / 12.8;
		controller->x = (((float) controller->a[1]) - 512.0) / 12.8;
		controller->r = -1 * ((((float) controller->a[2]) - 512.0) / 51.2);
	}

	gait_process(gait);

	if(controller->r == 0.0)
	{
		// Start with neutral foot position		
		for(int i = 0; i < NUM_LEGS; i++)
		{
			goal.foot[i].x = neutral.foot[i].x;
			goal.foot[i].y = neutral.foot[i].y;
			goal.foot[i].z = neutral.foot[i].z;
		}
	}
	else
	{
		// start with rotations from neutral foot position
		float theta;
		float costheta;
		float sintheta;
		float xfromcenter;
		float yfromcenter;
		
		for(int i = 0; i < NUM_LEGS; i++)
		{
			theta = gait->r[i] * controller->r * 0.01745;
			costheta = cos(theta);
			sintheta = sin(theta);
			xfromcenter = neutral.foot[i].x + coxaoffset.foot[i].x;
			yfromcenter = neutral.foot[i].y + coxaoffset.foot[i].y;

			goal.foot[i].x = (xfromcenter * costheta - yfromcenter * sintheta) - coxaoffset.foot[i].x;
			goal.foot[i].y = (xfromcenter * sintheta + yfromcenter * costheta) - coxaoffset.foot[i].y;
			goal.foot[i].z = neutral.foot[i].z;
		}
	}
		
	// Add gait translations to goal position
	for(int i = 0; i < NUM_LEGS; i++)
	{
		goal.foot[i].x += gait->x[i] * controller->x;
		goal.foot[i].y += gait->y[i] * controller->y;
		goal.foot[i].z += gait->z[i] * controller->z;
	}
		
	// Add gait body shift to goal position if required.
	if(controller->s != 0.0)
	{
		gait_shift_process(gait);
		for(int i = 0; i < NUM_LEGS; i++)
		{
			goal.foot[i].x -= gait->sx * controller->s;
			goal.foot[i].y -= gait->sy * controller->s;
		}
	}

	gait_increment(gait);

	for(uint8_t i = 0; i < NUM_LEGS; i++)
		kinematics_legik(goal.foot[i].x, goal.foot[i].y, goal.foot[i].z, &joint[i*4].angle, &joint[i*4+1].angle, &joint[i*4+2].angle, &joint[i*4+3].angle);

	for(uint8_t i = 0; i < NUM_TURRETS; i++)
		kinematics_turretik();

	for(uint8_t i = 0; i < NUM_GUNS; i++)
		kinematics_gunik();

	joint_write(&joint);
}
