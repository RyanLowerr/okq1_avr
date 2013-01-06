
#include <math.h>

#include "motionmanager.h"
#include "types.h"
#include "controller.h"
#include "common.h"
#include "position.h"
#include "kinematics.h"
#include "joint.h"
#include "okmath.h"
#include "gait.h"
#include "dynamixel.h"
#include "mx.h"
#include "ax.h"

void mm_init(void)
{
	;
}

void mm_process(CONTROLLER *controller, GAIT *gait)
{
	/*
	if(controller_buffer_size() >= 19)
	{
		controller_read(controller);
		controller->s = (u08) (((float)controller->a[3]) / 17.0); 
	}
	
	if(gait->position == 0)
	{
		controller->y = (((float) controller->a[0]) - 512.0) / 12.8;
		controller->x = (((float) controller->a[1]) - 512.0) / 12.8;
		controller->r = -1 * ((((float) controller->a[2]) - 512.0) / 51.2);
		controller->s = (u08) (((float)controller->a[3]) / 17.0);
	}
	*/
	controller->y = 20.0;
	controller->x = 0.0;
	controller->z = 30.0;

	gait_process(gait);

	for(u08 i = 0; i < NUM_LEGS; i++)
	{
		if(controller->r == 0.0)
		{
			// Start with neutral foot position		
			goal.foot[i].x = neutral.foot[i].x;
			goal.foot[i].y = neutral.foot[i].y;
			goal.foot[i].z = neutral.foot[i].z;
		}
		else
		{
			// start with rotations from neutral foot position
			float theta;
			float costheta;
			float sintheta;
			float xfromcenter;
			float yfromcenter;
		
			theta = gait->tran[i] * controller->r * 0.01745;
			costheta = cos(theta);
			sintheta = sin(theta);
			xfromcenter = neutral.foot[i].x + coxaoffset.foot[i].x;
			yfromcenter = neutral.foot[i].y + coxaoffset.foot[i].y;

			goal.foot[i].x = (xfromcenter * costheta - yfromcenter * sintheta) - coxaoffset.foot[i].x;
			goal.foot[i].y = (xfromcenter * sintheta + yfromcenter * costheta) - coxaoffset.foot[i].y;
			goal.foot[i].z = neutral.foot[i].z;
		}
		
		// Add gait translations to goal position
		goal.foot[i].x += 1.0 * ((float)gait->tran[i] / 10000.0) * controller->x;
		goal.foot[i].y += 1.0 * ((float)gait->tran[i] / 10000.0) * controller->y;
		goal.foot[i].z += 1.0 * ((float)gait->lift[i] / 10000.0) * controller->z;
	}

	gait_increment(gait, 400);

	for(u08 i = 0; i < NUM_LEGS; i++)
		kinematics_leg_ik(goal.foot[i].x, goal.foot[i].y, goal.foot[i].z, &joint[i*4].angle, &joint[i*4+1].angle, &joint[i*4+2].angle, &joint[i*4+3].angle);

	for(u08 i = 0; i < NUM_TURRETS; i++)
		kinematics_turret_ik();

	for(u08 i = 0; i < NUM_GUNS; i++)
		kinematics_gun_ik();

	joint_write(&joint[0]);
}
