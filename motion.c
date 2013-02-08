
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
	m->leg_state = MOTION_LEGS_WALKING;
	m->turret_state = 0;
	m->gun_state = 0;
}

void motion_set_leg_state(MOTION *m, u16 state)
{
	m->leg_state = state;
}

void motion_set_turret_state(MOTION *m, u16 state)
{
	m->turret_state = state;
}

void motion_set_gun_state(MOTION *m, u16 state)
{
	m->gun_state = state;
}

static void motion_legs_walking(void)
{
	gait_process(&gait);
	
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

static motion_legs_tracking(void)
{
}

void motion_process(MOTION *m)
{	
	// Calculate the legs goal positions.
	switch(m->leg_state)
	{
		case MOTION_LEGS_INTERPOLATING:
			if(interpolation_step(&interp_legs, &goal, 500))
				motion_set_leg_state(m, MOTION_LEGS_IDLING);
			break;
			
		case MOTION_LEGS_WALKING:
			motion_legs_walking();
			break;
			
		case MOTION_LEGS_TRACKING:
			motion_legs_tracking();
			break;
			
		case MOTION_LEGS_IDLING:
			m->leg_idle_count += 1;
			break;
	}
	
	// Calculate the turret(s) goal position(s).
	switch(m->turret_state)
	{
		case MOTION_TURRETS_INTERPOLATING:
			if(interpolation_step(&interp_turrets, &goal, 500))
				motion_set_turret_state(m, MOTION_TURRETS_IDLING);
			break;
			
		case MOTION_TURRETS_FOLLOWING:
			break;
			
		case MOTION_TURRETS_TRACKING:
			break;
			
		case MOTION_TURRETS_IDLING:
			m->turret_idle_count += 1;
			break;
	}
	
	// Calculate the gun(s) goal position(s).
	switch(m->gun_state)
	{
		case MOTION_GUNS_INTERPOLATING:
			if(interpolation_step(&interp_guns, &goal, 500))
				motion_set_gun_state(m, MOTION_GUNS_IDLING);
			break;
			
		case MOTION_GUNS_FOLLOWING:
			break;
			
		case MOTION_GUNS_TRACKING:
			break;
			
		case MOTION_GUNS_IDLING:
			m->gun_idle_count += 1;
			break;
	}
	
	// Perform the IK on all legs, turrets and guns using the caclulated goal positon.
	for(u08 i = 0; i < NUM_LEGS; i++)
		kinematics_leg_ik(goal.foot[i].x, goal.foot[i].y, goal.foot[i].z, &joint[i*4].angle, &joint[i*4+1].angle, &joint[i*4+2].angle, &joint[i*4+3].angle);
	
	for(u08 i = 0; i < NUM_TURRETS; i++)
		kinematics_turret_ik();
	
	for(u08 i = 0; i < NUM_GUNS; i++)
		kinematics_gun_ik();
	
	// Save the previous goal potsition as the robot's current position.
	current = goal;
}