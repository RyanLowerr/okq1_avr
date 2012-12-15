
#ifndef _GAIT_H_
#define _GAIT_H_

#include <avr/io.h>

#define GAIT_TYPE_RIPPLE  1
#define GAIT_TYPE_AMBLE   2

typedef struct
{
	uint8_t type;

	float period;
	float periodstep;
	uint8_t position;
	
	float step_to_move_ratio;
	float move_to_step_ratio;
	float step_time;
	float move_time;

	float step_period_x;
	float step_period_y;
	float step_period_z;

	float step_periodshift_x;
	float step_periodshift_y;
	float step_periodshift_z;

	float move_period_x;
	float move_period_y;
	float move_period_z;

	float move_periodshift_x;
	float move_periodshift_y;
	float move_periodshift_z;

	float step_start[4];
	float step_end[4];
	
	float x[4];
	float y[4];
	float z[4];
	float r[4];
	float sx;
	float sy;
} GAIT;

extern GAIT gait;

void gait_process(GAIT *g);				
void gait_shift_process(GAIT *g);
void gait_increment(GAIT *g);
void gait_paramcalc(GAIT *g);
void gait_init(GAIT *g, uint8_t type);
float gait_sine(float position, float period, float period_shift, float amplitude, float amplitude_shift);
float gait_line(float position, float period, float period_shift, float amplitude, float amplitude_shift);

#endif
