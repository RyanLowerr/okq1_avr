
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
	
	m->look_s   = 500;
	m->aim_s    = 500;
	 
	m->state_leg    = MOTIONSTATE_LEGS_INIT;
	m->state_turret = MOTIONSTATE_TURRETS_INIT;
	m->state_gun    = MOTIONSTATE_GUNS_INIT;
	
	m->status_leg    = MOTIONSTATUS_LEGS_IDLING;
	m->status_turret = MOTIONSTATUS_TURRETS_FOLLOWING;
	m->status_gun    = MOTIONSTATUS_GUNS_FOLLOWING;
	
	m->idle_count = 0;
}

static void motion_paramaters(MOTION *m, CONTROLLER *c)
{
	m->travel_x = 0;
	m->travel_y = 400;
	m->travel_r = 0;
	m->travel_l = 250;
	m->travel_s = 500;
	m->travel_request = 1;
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
		if(((m->status_leg != MOTIONSTATUS_LEGS_INTERPOLATING) && (m->status_leg != MOTIONSTATUS_LEGS_WALKING)) ||
		   ((m->status_leg == MOTIONSTATUS_LEGS_WALKING) && (m->travel_largechange)))
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
		else if((m->idle_count >= 100) && (m->state_leg != MOTIONSTATE_LEGS_SIT))
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
	
	/*
	 *   Gun state controls.
	 */
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
			
			if(interpolation_step(&interp_legs, &goal, 700))
			{
				if(m->state_leg & MOTIONSTATE_LEGS_WALK)
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
			m->idle_count += 1;
			break;
	}
	
	switch(m->status_gun)
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