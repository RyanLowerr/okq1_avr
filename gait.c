
#include <math.h>

#include "gait.h"
#include "types.h"
#include "okmath.h"

GAIT gait;

static void gait_paramcalc(GAIT *g)
{
	g->period = MAX_U16;
	g->step_period = ((u32)g->period * (u32)g->step_to_move_ratio) / DEC2;
	g->move_period = g->period - g->step_period;
	
	for(u08 i = 0; i < 4; i++)
	{
		g->step_start[i] = ((u32)g->period * (u32)g->start_position[i]) / DEC2;
		g->step_end[i] = g->step_start[i] + g->step_period;
	}
}

void gait_init(GAIT *g, u08 type)
{
	g->position = 0;

	if(type == GAIT_TYPE_RIPPLE)
	{
		g->step_to_move_ratio = 25;
		g->start_position[0] = 0;
		g->start_position[1] = 25;
		g->start_position[2] = 50;
		g->start_position[3] = 75;
		gait_paramcalc(g);
	}
	
	if(type == GAIT_TYPE_AMBLE)
	{
		g->step_to_move_ratio = 50;
		g->start_position[0] = 0;
		g->start_position[1] = 50;
		g->start_position[2] = 0;
		g->start_position[3] = 50;
		gait_paramcalc(g);
	}
}

void gait_process(GAIT *g)
{
	s32 shiftedposition = 0;

	// For each leg calculate percentage of leg endpoint movement
	for(u08 i = 0; i < 4; i++)
	{	
		shiftedposition = g->position - g->step_start[i];
		if(shiftedposition < 0)
			shiftedposition += g->period;
			
		// Foot is raised and lowerd while the leg moves forward.
		if((g->position >= g->step_start[i]) && (g->position <= g->step_end[i]))
		{
			// Translation is movement along a line from -1 (fully backwards) to 1 (full forward) for this portion of the gait.
			// A rearanged and simplified point slope format equation for a line (y = mx + b) is used here.
			g->tran[i] = ((shiftedposition * 2 * DEC4) / g->step_period) - DEC4;			
			
			// Lift follows the first 180 degrees of a sin wave for this portion of the gait. (0.0 -> 1.0 -> 0.0)
			g->lift[i] = okmath_sin((shiftedposition * 180 * DEC1) / g->step_period);
		}
		
		// Foot is on the ground an leg moves backwards to move the boday forward.
		else
		{			
			// Translation is movement along a line from 1 (full forward) to -1 (fully backwards) for this portion of the gait.
			// A rearanged and simplified point slope format equation for a line (y = mx + b) is used here.
			g->tran[i] = -((((shiftedposition - g->step_period) * 2 * DEC4) / g->move_period) - DEC4);
			
			// Lift is always zero for this portion of the gait.
			g->lift[i] = 0;
		}
	}
}

void gait_increment(GAIT *g, u16 step_size)
{	
	if((u32)g->position + step_size > g->period)
		g->position = 0;
	else
		g->position += step_size;			
}

void gait_decrement(GAIT *g, u16 step_size)
{	
	if((s32)g->position - step_size < 0)
		g->position = g->period;
	else
		g->position -= step_size;
}
