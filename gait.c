
#include <avr/io.h>
#include <math.h>

#include "gait.h"

GAIT gait;

static void gait_paramcalc(GAIT *g)
{
	g->move_to_step_ratio = 1.0 - g->step_to_move_ratio;
	g->step_time = g->period * g->step_to_move_ratio;
	g->move_time = g->period * g->move_to_step_ratio;
	
	g->step_period_x = g->step_time;
	g->step_period_y = g->step_time;
	g->step_period_z = g->step_time * 2.0;
	
	g->step_periodshift_x = 0.0;
	g->step_periodshift_y = 0.0;
	g->step_periodshift_z = 0.0;
	
	g->move_period_x = g->period * g->move_to_step_ratio;
	g->move_period_y = g->period * g->move_to_step_ratio;
	g->move_period_z = 0.0;
	
	g->move_periodshift_x = -g->step_period_x;
	g->move_periodshift_y = -g->step_period_y;
	g->move_periodshift_z = 0.0;
	
	g->step_end[0] = g->step_start[0] + g->step_time;
	g->step_end[1] = g->step_start[1] + g->step_time;
	g->step_end[2] = g->step_start[2] + g->step_time;
	g->step_end[3] = g->step_start[3] + g->step_time;
}

static float gait_sine(float position, float period, float period_shift, float amplitude, float amplitude_shift)
{
	return (amplitude * sin(2 * 3.141592 / period * (position + period_shift)) + amplitude_shift);
}

static float gait_line(float position, float period, float period_shift, float amplitude, float amplitude_shift)
{
	return ((position + period_shift) * (amplitude / period) + amplitude_shift);
}

void gait_init(GAIT *g, uint8_t type)
{

	// This should be configurable. Possibly a param of gait_init()?
	g->period = 3000;
	g->position = 0;

	if(type == GAIT_TYPE_RIPPLE)
	{
		g->type = type;
		g->step_to_move_ratio = 0.25;
		g->step_start[0] = g->period * 0.00;
		g->step_start[1] = g->period * 0.25;
		g->step_start[2] = g->period * 0.50;
		g->step_start[3] = g->period * 0.75;
		gait_paramcalc(g);
	}
	
	if(type == GAIT_TYPE_AMBLE)
	{
		g->type = type;
		g->step_to_move_ratio = 0.50;
		g->step_start[0] = g->period * 0.00;
		g->step_start[1] = g->period * 0.50;
		g->step_start[2] = g->period * 0.00;
		g->step_start[3] = g->period * 0.50;
		gait_paramcalc(g);
	}
}

void gait_process(GAIT *g)
{	
	float shiftedtime;
	
	// for each leg calculate percentage of leg endpoint movement
	for(uint8_t legindex = 0; legindex < 4; legindex++)
	{	
		// shift start time to 0
		shiftedtime = (float) g->position - g->step_start[legindex];
		if(shiftedtime < 0)
			shiftedtime += g->period;
	
		// Leg lift and forward
		if((g->position >= g->step_start[legindex]) && (g->position <= g->step_end[legindex]))
		{
			g->z[legindex] = gait_sine(shiftedtime, g->step_period_z, g->step_periodshift_z, 1.0,  0.0);
			g->x[legindex] = gait_line(shiftedtime, g->step_period_x, g->step_periodshift_x, 2.0, -1.0);
			g->y[legindex] = g->x[legindex];
			g->r[legindex] = g->x[legindex];
		}
		
		// Leg moves boday forward
		else
		{
			g->z[legindex] = 0.0;
			g->x[legindex] = gait_line(shiftedtime, g->move_period_x, g->move_periodshift_x, -2.0, 1.0);
			g->y[legindex] = g->x[legindex];
			g->r[legindex] = g->x[legindex];
		}
	}
}

void gait_shift_process(GAIT *g)
{
	float angle = (225.0 - (360.0 / g->period) * (float) g->position) * 0.01745;
	g->sx = cos(angle) - sin(angle);
	g->sy = sin(angle) + cos(angle);
}

void gait_increment(GAIT *g, uint8_t step_size)
{	
	if(g->position + step_size > g->period)
		g->position = 0;
	else
		g->position += step_size;			
}

void gait_decrement(GAIT *g, uint8_t step_size)
{	
	if(g->position - step_size < 0)
		g->position = g->period;
	else
		g->position -= step_size;
}
