
#include "motion.h"
#include "types.h"
#include "controller.h"
#include "common.h"
#include "position.h"
#include "interpolation.h"
#include "kinematics.h"
#include "okmath.h"
#include "gait.h"
#include "okmath.h"
#include "dynamixel.h"

// Motion Paramaters for walking, looking with the turret, and aiming the guns.
s16 travel_x; // Translational travel along the X axis in mm. DEC1
s16 travel_y; // Translational travel along the Y axis in mm. DEC1
s16 travel_r; // Rotational travel around the Z axis in degrees. DEC1
s16 travel_l; // Leg lift height in mm. DEC1
s16 travel_s; // Speed of travel.
u08 travel_request;
u08 travel_largechange;
	
u08 state_leg; 
u08 status_leg;
	
// Idle counters.
u16 idle_count;

static void motion_gait(GAIT *g, POSITION *p)
{
	for(u08 i = 0; i < NUM_LEGS; i++)
	{
		// start with rotations from neutral foot position
		s16 theta;
		s16 costheta;
		s16 sintheta;
		s16 xfromcenter;
		s16 yfromcenter;
			
		theta = ((s32)g->tran[i] * travel_r) / DEC4;
		costheta = okmath_cos(theta);
		sintheta = okmath_sin(theta);
		xfromcenter = neutral.foot[i].x + coxaoffset.foot[i].x;
		yfromcenter = neutral.foot[i].y + coxaoffset.foot[i].y;
			
		p->foot[i].x = (((s32)xfromcenter * costheta) / DEC4) - (((s32)yfromcenter * sintheta) / DEC4) - coxaoffset.foot[i].x;
		p->foot[i].y = (((s32)xfromcenter * sintheta) / DEC4) + (((s32)yfromcenter * costheta) / DEC4) - coxaoffset.foot[i].y;
		p->foot[i].z = neutral.foot[i].z;
			
		// Add gait translations to goal position
		p->foot[i].x += ((s32)g->tran[i] * travel_x) / DEC4;
		p->foot[i].y += ((s32)g->tran[i] * travel_y) / DEC4;
		p->foot[i].z += ((s32)g->lift[i] * travel_l) / DEC4;
	}
}

void motion_init(void)
{
	travel_x = 0;
	travel_y = 0;
	travel_r = 0;
	travel_l = 0;
	travel_s = 500;
	
	travel_request = 0;
	travel_largechange = 0;
	
	state_leg  = MOTIONSTATE_LEGS_INIT;
	status_leg = MOTIONSTATUS_LEGS_IDLING;
	
	idle_count = 0;
}

void motion_process(CONTROLLER *c)
{
	travel_s = 600; 
	travel_y = (c->analog[3] >= 128) ?  (-c->analog[3] + 255) * 3: -(c->analog[3]) * 3;
	travel_x = (c->analog[2] >= 128) ? -(-c->analog[2] + 255) * 3:  (c->analog[2]) * 3;
	travel_r = (c->analog[0] >= 128) ?  (-c->analog[0] + 255)/1.27 : -(c->analog[0]/1.27);
	travel_l = 350;
	
	travel_request = ((travel_x >= 20) || (travel_x <= -20) || (travel_y >= 20) || (travel_y <= -20) || (travel_r >= 10) || (travel_r <= -10)) ? 1 : 0;
	
	// We have a travel request.
	if(travel_request)
	{
		// We have a travel request. Setting leg state to walk.
		state_leg = MOTIONSTATE_LEGS_WALK;
		
		// If we are not interpolating or walking OR are walking and have had a large travel request change.
		if((status_leg != MOTIONSTATUS_LEGS_INTERPOLATING) && (status_leg != MOTIONSTATUS_LEGS_WALKING))
		{
			// Calculate the goal position to interpolate to.
			gait_process(&gait);
			motion_gait(&gait, &goal);
			
			// Initalize interpolation for the legs from the robots current position to the calculated goal position.
			// Then set the robots status as interpolating.
			interpolation_init(&interp, &current, &goal);
			status_leg = MOTIONSTATUS_LEGS_INTERPOLATING;
		}
	}
	// We do not have a travel request.
	else
	{		
		// If we are not interpolating and our state is not already stand or sit. 
		// Interpolate to a standing position with all four feet on the ground.
		if((status_leg != MOTIONSTATUS_LEGS_INTERPOLATING) && (state_leg != MOTIONSTATE_LEGS_STAND) && (state_leg != MOTIONSTATE_LEGS_SIT))
		{
			// Set leg state to stand.
			state_leg = MOTIONSTATE_LEGS_STAND;
			
			// set the goal position.
			for(u08 i = 0; i < NUM_LEGS; i++)
			{
				goal.foot[i].x = current.foot[i].x;
				goal.foot[i].y = current.foot[i].y;
				goal.foot[i].z = neutral.foot[i].z;
			}
			
			// Initalize interpolation for the legs from the robot's current postion to the neutral standing position.
			// Then set the robots status as interpolating.
			interpolation_init(&interp, &current, &goal);
			status_leg = MOTIONSTATUS_LEGS_INTERPOLATING;
		}
		// If we have been idle for a while initalize interpolation to the sitting position.
		else if((idle_count >= 500) && (state_leg != MOTIONSTATE_LEGS_SIT))
		{
			// Set leg state to stand.
			state_leg = MOTIONSTATE_LEGS_SIT;
			
			// Set the goal position.
			for(u08 i = 0; i < NUM_LEGS; i++)
			{
				goal.foot[i].x = neutral.foot[i].x;
				goal.foot[i].y = neutral.foot[i].y;
				goal.foot[i].z = FOOT_Z_SITTING;
			}
			
			// Initalize interpolation for the legs from the robot's current postion to the sitting position.
			// Then set the robots status as interpolating.
			interpolation_init(&interp, &current, &goal);
			status_leg = MOTIONSTATUS_LEGS_INTERPOLATING;
		}
	}
	
	switch(status_leg)
	{
		/*
		 *   The legs are interpolating into a new position.
		 */
		case MOTIONSTATUS_LEGS_INTERPOLATING:
			
			// We are not Idling. Reset the idle counter.
			idle_count = 0;
			
			if(interpolation_step(&interp, &goal, 600))
			{
				if(state_leg == MOTIONSTATE_LEGS_WALK)
					status_leg = MOTIONSTATUS_LEGS_WALKING;
				else
					status_leg = MOTIONSTATUS_LEGS_IDLING;
			}
			
			break;
		
		/*
		 *   The legs are walking using our gait routines and calculated travel paramaters.
		 */
		case MOTIONSTATUS_LEGS_WALKING:
			
			// We are not Idling. Reset the idle counter.
			idle_count = 0;
			
			// Increment the gait position and process.
			// Idealy we have already interpolated to the current position to get to this
			// point so we need to increment the gait to calculate our next goal position.
			gait_increment(&gait, travel_s);
			gait_process(&gait);
			
			// Calculate the legs goal positions.
			motion_gait(&gait, &goal);
			break;
		
		/*
		 *   The legs are idle so we increment an idle counter.
		 *   Idle counter can be used to flag or quee up new position interpolations.
		 *   EXAMPLE: If robot has been standing idle for X calls of motion_process() initalize
		 *            interpolation between current position and sitting position.
		 */
		case MOTIONSTATUS_LEGS_IDLING:
			
			if(idle_count != MAX_U16)
				idle_count += 1;
			else
				idle_count = MAX_U16;
			
			break;
	}
	
	// Perform the IK on all legs, turrets and guns using the caclulated goal positon.
	for(u08 i = 0; i < NUM_LEGS; i++)
		kinematics_leg_ik(goal.foot[i].x, goal.foot[i].y, goal.foot[i].z, &servo[i*4].angle, &servo[i*4+1].angle, &servo[i*4+2].angle, &servo[i*4+3].angle);
	
	// Save the previous goal potsition as the robot's current position.
	current = goal;
}
