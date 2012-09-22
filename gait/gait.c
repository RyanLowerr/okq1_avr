
#include <avr/math.h>

#include "gait.h"

void gait_process(void)
{	
	double shiftedtime;	
	int legindex = 0;
	
	if(position > period)
		position = 0;
	
	// for each leg calculate percentage of leg endpoint movement
	while(legindex < 4)
	{	
		// shift start time to 0
		shiftedtime = position - step_start[legindex];
		if(shiftedtime < 0)
			shiftedtime += period;
	
		// Leg lift and forward
		if((position >= step_start[legindex]) && (position <= step_end[legindex]))
		{
			x[legindex] = gait_line(shiftedtime, step_period_x, step_periodshift_x, -1.0, 0.0);
			y[legindex] = gait_line(shiftedtime, step_period_y, step_periodshift_y, -1.0, 0.0);
			z[legindex] = gait_sine(shiftedtime, step_period_z, step_periodshift_z,  1.0, 0.0);
		}
		
		// Leg moves boday forward
		else
		{
			shiftedtime -= step_time;
			x[legindex] = gait_line(shiftedtime, move_period_x, move_periodshift_x, 1.0, 0.0);
			y[legindex] = gait_line(shiftedtime, move_period_y, move_periodshift_y, 1.0, 0.0);
			z[legindex] = 0.0;
		}
		
		legindex++;
	}
	
	position += 1;
}

double gait_sine(double position, double Period, double period_shift, double amplitude, double amplitude_shift)
{
	return (amplitude * sin(2 * 3.141592 / Period * (position + period_shift)) + amplitude_shift);
}

double gait_line(double position, double Period, double period_shift, double amplitude, double amplitude_shift)
{
	return (amplitude + (position * ((-amplitude - amplitude) / Period)));
}

void gait_paramcalc(void)
{
	move_to_step_ratio = 1 - step_to_move_ratio;
	step_time = period * step_to_move_ratio;
	move_time = period * move_to_step_ratio;
	
	step_period_x = step_time;
	step_period_y = step_time;
	step_period_z = step_time * 2;
	
	move_period_x = period * move_to_step_ratio;
	move_period_y = period * move_to_step_ratio;
	move_period_z = 0;
	
	move_periodshift_x = -move_period_x * 0.25;
	move_periodshift_y = -move_period_y * 0.25;
	move_periodshift_z = 0;
	
	step_end[0] = step_start[0] + step_time;
	step_end[1] = step_start[1] + step_time;
	step_end[2] = step_start[2] + step_time;
	step_end[3] = step_start[3] + step_time;
}

/*
void rippleinit()
{
	step_to_move_ratio = 0.25;
	step_start[0] = period * 0.00;
	step_start[1] = period * 0.25;
	step_start[2] = period * 0.50;
	step_start[3] = period * 0.75;
	paramcalc();
}

void ambleinit()
{
	step_to_move_ratio = 0.50;
	step_start[0] = period * 0.00;
	step_start[1] = period * 0.50;
	step_start[2] = period * 0.00;
	step_start[3] = period * 0.50;
	paramcalc();
}
*/
