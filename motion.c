
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
	m->travel_x = 0;
	m->travel_y = 0;
	m->travel_r = 0;
	m->travel_l = 0;
	m->travel_s = 500;
	
	m->travel_request = 0;
	m->travel_largechange = 0;
	
	m->look_x = 0;
	m->look_y = 0;
	m->look_s = 500;
	
	m->aim_f = 0;
	m->aim_s = 500;
	 
	m->state_leg    = MOTIONSTATE_LEGS_INIT;
	m->state_turret = MOTIONSTATE_TURRETS_INIT;
	m->state_gun    = MOTIONSTATE_GUNS_INIT;
	
	m->status_leg    = MOTIONSTATUS_LEGS_IDLING;
	m->status_turret = MOTIONSTATUS_TURRETS_FOLLOWING;
	m->status_gun    = MOTIONSTATUS_GUNS_FOLLOWING;
	
	m->idle_count = 0;
	
	// HACK  HACK  HACK
	joint[17].angle = 450;
	joint[18].angle = 0;
	joint[19].angle = 0;
	joint[20].angle = 0;
}

static void motion_paramaters(MOTION *m, CONTROLLER *c)
{
	//m->travel_s = ((s32)c->analog[3] * DEC1) / 170; 
	m->travel_y = (((s32)c->analog[0] - 512) * DEC2) / 128;
	m->travel_x = (((s32)c->analog[1] - 512) * DEC2) / 128;
	m->travel_r = 1 - ((((s32)c->analog[2] - 512) * DEC2) / 500);
	m->travel_l = 300;
	
	if((m->travel_x >= 20) || (m->travel_x <= -20) || (m->travel_y >= 20) || (m->travel_y <= -20) || (m->travel_r >= 20) || (m->travel_r <= -20))
		m->travel_request = 1;
	else
		m->travel_request = 0;
	
	m->look_x = (((s32)c->analog[3] - 512) * DEC2) / 8192;
	m->look_y = (((s32)c->analog[4] - 512) * DEC2) / 8192;
	//m->look_s;
	
	if((m->look_x >= 20) || (m->look_x <= -20) || (m->look_y >= 20) || (m->look_y <= -20))
		m->look_request = 1;
	else
		m->look_request = 0;
	
	m->aim_f = (((s32)c->analog[5] - 512) * DEC2) / 8192;
	//m->aim_s;
	
	if((m->aim_f >= 20) || (m->aim_f <= -20))
		m->aim_request = 1;
	else
		m->aim_request = 0;
}

static void motion_gait(MOTION *m, GAIT *g, POSITION *p)
{
	for(u08 i = 0; i < NUM_LEGS; i++)
	{
		// start with rotations from neutral foot position
		s16 theta;
		s16 costheta;
		s16 sintheta;
		s16 xfromcenter;
		s16 yfromcenter;
			
		theta = ((s32)g->tran[i] * m->travel_r) / DEC4;
		costheta = okmath_cos(theta);
		sintheta = okmath_sin(theta);
		xfromcenter = neutral.foot[i].x + coxaoffset.foot[i].x;
		yfromcenter = neutral.foot[i].y + coxaoffset.foot[i].y;
			
		p->foot[i].x = (((s32)xfromcenter * costheta) / DEC4) - (((s32)yfromcenter * sintheta) / DEC4) - coxaoffset.foot[i].x;
		p->foot[i].y = (((s32)xfromcenter * sintheta) / DEC4) + (((s32)yfromcenter * costheta) / DEC4) - coxaoffset.foot[i].y;
		p->foot[i].z = neutral.foot[i].z;
			
		// Add gait translations to goal position
		p->foot[i].x += ((s32)g->tran[i] * m->travel_x) / DEC4;
		p->foot[i].y += ((s32)g->tran[i] * m->travel_y) / DEC4;
		p->foot[i].z += ((s32)g->lift[i] * m->travel_l) / DEC4;
	}
}

static void motion_state(MOTION *m)
{
	/*
	 *   Leg state controls.
	 */
	
	// We have a travel request.
	if(m->travel_request)
	{
		// We have a travel request. Setting leg state to walk.
		m->state_leg = MOTIONSTATE_LEGS_WALK;
		
		// If we are not interpolating or walking OR are walking and have had a large travel request change.
		if((m->status_leg != MOTIONSTATUS_LEGS_INTERPOLATING) && (m->status_leg != MOTIONSTATUS_LEGS_WALKING))
		{
			// Calculate the goal position to interpolate to.
			gait_process(&gait);
			motion_gait(m, &gait, &goal);
			
			// Initalize interpolation for the legs from the robots current position to the calculated goal position.
			// Then set the robots status as interpolating.
			interpolation_init(&interp_legs, &current, &goal, INTERPOLATION_IGNORE_TURRETS + INTERPOLATION_IGNORE_GUNS);
			m->status_leg = MOTIONSTATUS_LEGS_INTERPOLATING;
		}
	}
	// We do not have a travel request.
	else
	{		
		// If we are not interpolating and our state is not already stand or sit. 
		// Interpolate to a standing position with all four feet on the ground.
		if((m->status_leg != MOTIONSTATUS_LEGS_INTERPOLATING) && (m->state_leg != MOTIONSTATE_LEGS_STAND) && (m->state_leg != MOTIONSTATE_LEGS_SIT))
		{
			// Set leg state to stand.
			m->state_leg = MOTIONSTATE_LEGS_STAND;
			
			// set the goal position.
			for(u08 i = 0; i < NUM_LEGS; i++)
			{
				goal.foot[i].x = current.foot[i].x;
				goal.foot[i].y = current.foot[i].y;
				goal.foot[i].z = neutral.foot[i].z;
			}
			
			// Initalize interpolation for the legs from the robot's current postion to the neutral standing position.
			// Then set the robots status as interpolating.
			interpolation_init(&interp_legs, &current, &goal, INTERPOLATION_IGNORE_TURRETS + INTERPOLATION_IGNORE_GUNS);
			m->status_leg = MOTIONSTATUS_LEGS_INTERPOLATING;
		}
		// If we have been idle for a while initalize interpolation to the sitting position.
		else if((m->idle_count >= 500) && (m->state_leg != MOTIONSTATE_LEGS_SIT))
		{
			// Set leg state to stand.
			m->state_leg = MOTIONSTATE_LEGS_SIT;
			
			// Set the goal position.
			for(u08 i = 0; i < NUM_LEGS; i++)
			{
				goal.foot[i].x = neutral.foot[i].x;
				goal.foot[i].y = neutral.foot[i].y;
				goal.foot[i].z = FOOT_Z_SITTING;
			}
			
			// Initalize interpolation for the legs from the robot's current postion to the sitting position.
			// Then set the robots status as interpolating.
			interpolation_init(&interp_legs, &current, &goal, INTERPOLATION_IGNORE_TURRETS + INTERPOLATION_IGNORE_GUNS);
			m->status_leg = MOTIONSTATUS_LEGS_INTERPOLATING;
		}
	}
	
	/*
	 *   Turret state controls.
	 */
	if(m->look_request)
	{
	}
	else
	{
	}
	
	/*
	 *   Gun state controls.
	 */
	if(m->aim_request)
	{
	}
	else
	{
	}
}

static void motion_status(MOTION *m)
{
	switch(m->status_leg)
	{
		/*
		 *   The legs are interpolating into a new position.
		 */
		case MOTIONSTATUS_LEGS_INTERPOLATING:
			
			// We are not Idling. Reset the idle counter.
			m->idle_count = 0;
			
			if(interpolation_step(&interp_legs, &goal, 600))
			{
				if(m->state_leg == MOTIONSTATE_LEGS_WALK)
					m->status_leg = MOTIONSTATUS_LEGS_WALKING;
				else
					m->status_leg = MOTIONSTATUS_LEGS_IDLING;
			}
			
			break;
		
		/*
		 *   The legs are walking using our gait routines and calculated travel paramaters.
		 */
		case MOTIONSTATUS_LEGS_WALKING:
			
			// We are not Idling. Reset the idle counter.
			m->idle_count = 0;
			
			// Increment the gait position and process.
			// Idealy we have already interpolated to the current position to get to this
			// point so we need to increment the gait to calculate our next goal position.
			gait_increment(&gait, m->travel_s);
			gait_process(&gait);
			
			// Calculate the legs goal positions.
			motion_gait(m, &gait, &goal);
			
			break;
		
		/*
		 *   The legs are idle so we increment an idle counter.
		 *   Idle counter can be used to flag or quee up new position interpolations.
		 *   EXAMPLE: If robot has been standing idle for X calls of motion_process() initalize
		 *            interpolation between current position and sitting position.
		 */
		case MOTIONSTATUS_LEGS_IDLING:
			if(m->idle_count != MAX_U16)
				m->idle_count += 1;
			else
				m->idle_count = MAX_U16;
			break;
	}
	
	switch(m->status_turret)
	{
		/*
		 *   The turrets are interpolating into a new position.
		 */
		case MOTIONSTATUS_TURRETS_INTERPOLATING:
			if(interpolation_step(&interp_turrets, &goal, m->look_s))
				m->status_turret = MOTIONSTATUS_TURRETS_IDLING;
			break;
			
		/*
		 *   The turrets are folling controller input.
		 */
		case MOTIONSTATUS_TURRETS_FOLLOWING:
			// HACK  HACK  HACK
			//goal.turret.x += current.turret.x + m->look_x;
			//goal.turret.y += current.turret.y + m->look_y;
			joint[16].angle += m->look_x * 1;
			if (joint[16].angle > PAN_MAX) joint[16].angle = PAN_MAX;
			if (joint[16].angle < PAN_MIN) joint[16].angle = PAN_MIN;
			
			joint[17].angle += m->look_y * 1;
			if (joint[17].angle > TILT_MAX) joint[17].angle = TILT_MAX;
			if (joint[17].angle < TILT_MIN) joint[17].angle = TILT_MIN;
			break;
			
		/*
		 *   The turrets are idle.
		 */
		case MOTIONSTATUS_TURRETS_TRACKING:
			break;
	}
	
	switch(m->status_gun)
	{
		/*
		 *   The guns are interpolating into a new position.
		 */
		case MOTIONSTATUS_GUNS_INTERPOLATING:
			if(interpolation_step(&interp_guns, &goal, m->aim_s))
				m->status_gun = MOTIONSTATUS_GUNS_IDLING;
			break;
			
		/*
		 *   The guns are folling controller input.
		 */
		case MOTIONSTATUS_GUNS_FOLLOWING:
			// HACK  HACK  HACK
			//goal.guns.z += current.turret.z + m->look_y;
			joint[18].angle += m->aim_f * 1;
			if (joint[18].angle > GUN_MAX) joint[18].angle = GUN_MAX;
			if (joint[18].angle < GUN_MIN) joint[18].angle = GUN_MIN;
			joint[19].angle = -joint[20].angle;
			break;
		
		/*
		 *   The guns are Idle.
		 */
		case MOTIONSTATUS_GUNS_TRACKING:
			break;
	}
}

void motion_process(MOTION *m, CONTROLLER *c)
{	
	motion_paramaters(m, c);
	motion_state(m);
	motion_status(m);
	
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