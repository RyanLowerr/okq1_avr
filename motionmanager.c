
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
#include "okmath.h"

MOTIONMANAGER motionmanager;

void mm_init(void)
{
	motionmanager.state = 0;
}

void mm_process(CONTROLLER *controller, GAIT *gait)
{
	if(controller_buffer_size() >= 19)
	{
		controller_read(controller);
		controller->s = ((s32)controller->a[3] * DEC1) / 170; 
		controller->y = (((s32)controller->a[0] - 512) * DEC2) / 128;
		controller->x = (((s32)controller->a[1] - 512) * DEC2) / 128;
		controller->r = 1 - ((((s32)controller->a[2] - 512) * DEC2) / 500);
		controller->z = 250;
	}
	gait_process(gait);

	/*
	switch(motionmanager.state)
	{
		//
		case MOTION_POWER_UP:
			motionmanager_setstate(MOTION_INTERPOLATE);
			break;
		
		//
		case MOTION_INTERPOLATE:
			
			if(&interpolation)

			break;
		
		//
		case MOTION_INTERPOLATE_DONE:
			break;
		
		//
		case MOTION_GAIT:
			break;
		
		//
		case MOTION_LEGS_DOWN:
			motionmanager_setstate(MOTION_INTERPOLATE);
			break;
		
		//
		case MOTION_LEGS_UP:
			motionmanager_setstate(MOTION_INTERPOLATE);
			break;
		
		//
		case MOTION_LEGS_NEUTRAL:
			motionmanager_setstate(MOTION_INTERPOLATE);
			break;
	}
	*/

	for(u08 i = 0; i < NUM_LEGS; i++)
	{
		if(controller->r == 0)
		{
			// Start with neutral foot position		
			goal.foot[i].x = neutral.foot[i].x;
			goal.foot[i].y = neutral.foot[i].y;
			goal.foot[i].z = neutral.foot[i].z;
		}
		else
		{
			// start with rotations from neutral foot position
			s16 theta;
			s16 costheta;
			s16 sintheta;
			s16 xfromcenter;
			s16 yfromcenter;
			
			theta = ((s32)gait->tran[i] * controller->r) / DEC4;
			costheta = okmath_cos(theta);
			sintheta = okmath_sin(theta);
			xfromcenter = neutral.foot[i].x + coxaoffset.foot[i].x;
			yfromcenter = neutral.foot[i].y + coxaoffset.foot[i].y;

			goal.foot[i].x = (((s32)xfromcenter * costheta) / DEC4) - (((s32)yfromcenter * sintheta) / DEC4) - coxaoffset.foot[i].x;
			goal.foot[i].y = (((s32)xfromcenter * sintheta) / DEC4) + (((s32)yfromcenter * costheta) / DEC4) - coxaoffset.foot[i].y;
			goal.foot[i].z = neutral.foot[i].z;
		}
			
		// Add gait translations to goal position
		goal.foot[i].x += ((s32)gait->tran[i] * controller->x) / DEC4;
		goal.foot[i].y += ((s32)gait->tran[i] * controller->y) / DEC4;
		goal.foot[i].z += ((s32)gait->lift[i] * controller->z) / DEC4;
	}

	gait_increment(gait, 500);

	for(u08 i = 0; i < NUM_LEGS; i++)
		kinematics_leg_ik(goal.foot[i].x, goal.foot[i].y, goal.foot[i].z, &joint[i*4].angle, &joint[i*4+1].angle, &joint[i*4+2].angle, &joint[i*4+3].angle);

	/*
	for(u08 i = 0; i < NUM_TURRETS; i++)
		kinematics_turret_ik();

	for(u08 i = 0; i < NUM_GUNS; i++)
		kinematics_gun_ik();
	*/
	
	joint_write(&joint[0]);

	// Copy the output goal potsition as the robot's current position.
	position_copy(&goal, &current);
}
