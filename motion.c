
#include "motion.h"
#include "types.h"
#include "controller.h"
#include "common.h"
#include "position.h"
#include "interpolation.h"
#include "kinematics.h"
#include "joint.h"
#include "okmath.h"
#include "gait.h"
#include "dynamixel.h"
#include "mx.h"
#include "ax.h"
#include "okmath.h"

MOTION motion;

void motion_init(MOTION *m)
{
	m->leg_state = 0;
	m->turret_state = 0;
	m->gun_state = 0;
}

static void motion_leg_walking(void)
{
	for(u08 i = 0; i < NUM_LEGS; i++)
	{
		if(controller.r == 0)
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
			
			theta = ((s32)gait.tran[i] * controller.r) / DEC4;
			costheta = okmath_cos(theta);
			sintheta = okmath_sin(theta);
			xfromcenter = neutral.foot[i].x + coxaoffset.foot[i].x;
			yfromcenter = neutral.foot[i].y + coxaoffset.foot[i].y;
			
			goal.foot[i].x = (((s32)xfromcenter * costheta) / DEC4) - (((s32)yfromcenter * sintheta) / DEC4) - coxaoffset.foot[i].x;
			goal.foot[i].y = (((s32)xfromcenter * sintheta) / DEC4) + (((s32)yfromcenter * costheta) / DEC4) - coxaoffset.foot[i].y;
			goal.foot[i].z = neutral.foot[i].z;
		}
			
		// Add gait translations to goal position
		goal.foot[i].x += ((s32)gait.tran[i] * controller.x) / DEC4;
		goal.foot[i].y += ((s32)gait.tran[i] * controller.y) / DEC4;
		goal.foot[i].z += ((s32)gait.lift[i] * controller.z) / DEC4;
	}
	
	gait_increment(&gait, 500);
}

static motion_leg_tracking(void)
{
}

void motion_process(MOTION *m)
{
	gait_process(&gait);
	
	switch(m->leg_state)
	{
		case MOTION_LEG_INTERPOLATING:
			break;
			
		case MOTION_LEG_WALKING:
			motion_leg_walking();
			break;
			
		case MOTION_LEG_TRACKING:
			motion_leg_tracking();
			break;
			
		case MOTION_LEG_IDLING:
			m->leg_idle_count += 1;
			break;
	}
	
	switch(m->turret_state)
	{
		case MOTION_TURRET_INTERPOLATING:
			break;
			
		case MOTION_TURRET_WALKING:
			break;
			
		case MOTION_TURRET_TRACKING:
			break;
			
		case MOTION_TURRET_IDLING:
			m->turret_idle_count += 1;
			break;
	}
	
	switch(m->gun_state)
	{
		case MOTION_GUN_INTERPOLATING:
			break;
			
		case MOTION_GUN_WALKING:
			break;
			
		case MOTION_GUN_TRACKING:
			break;
			
		case MOTION_GUN_IDLING:
			m->gun_idle_count += 1;
			break;
	}
	
	motion_leg_walking();
	
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