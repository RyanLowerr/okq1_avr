
#ifndef _GAIT_H_
#define _GAIT_H_

#include <avr/io.h>

#define GAIT_TYPE_RIPPLE  1
#define GAIT_TYPE_AMBLE   2

typedef struct
{
	uint8_t type;

	double period;
	double periodstep;
	double position;
	
	double step_to_move_ratio;
	double move_to_step_ratio;
	double step_time;
	double move_time;

	double step_period_x;
	double step_period_y;
	double step_period_z;

	double step_periodshift_x;
	double step_periodshift_y;
	double step_periodshift_z;

	double move_period_x;
	double move_period_y;
	double move_period_z;

	double move_periodshift_x;
	double move_periodshift_y;
	double move_periodshift_z;

	double step_start[4];
	double step_end[4];
	
	double x[4];
	double y[4];
	double z[4];
} gaitdata;
				
void gait_process(gaitdata *g);
void gait_paramcalc(gaitdata *g);
void gait_init(gaitdata *g, uint8_t type);
double gait_sine(double position, double Period, double period_shift, double amplitude, double amplitude_shift);
double gait_line(double position, double Period, double period_shift, double amplitude, double amplitude_shift);

#endif
