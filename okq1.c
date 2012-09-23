
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#include "dynamixel/dynamixel.h"
#include "dynamixel/ax.h"
#include "ik/ik.h"
#include "gait/gait.h"

int main(void)
{
	ik_angles leg_angles[4];
	gaitdata gait;	
	double x, y, z;
	
	dynamixel_init();
	gait_ripple_init(&gait);
	
	while(1)
	{		
		gait_process(&gait);
		
		x = 40.0 + gait.x[0] * 0.0;
		y = 100.0 + gait.y[0] * 0.0;
		z = -30.0 + gait.z[0] * 0.0;
		
		ik_leg(x, y, z, &leg_angles[0]);
	
		leg_angles[0].coxa   = (int) (AX_CENTER_VALUE + (leg_angles[0].coxa - 45.0) * 3.41);
		leg_angles[0].femur  = (int) (AX_CENTER_VALUE + (leg_angles[0].femur + 11.0) * 3.41);
		leg_angles[0].tibia  = (int) (AX_CENTER_VALUE - (leg_angles[0].tibia - 11.0) * 3.41);
		leg_angles[0].tarsus = (int) (AX_CENTER_VALUE - leg_angles[0].tarsus * 3.41);
	
		dynamixel_writeword(1, AX_GOAL_POSITION_L, leg_angles[0].coxa);
		dynamixel_writeword(2, AX_GOAL_POSITION_L, leg_angles[0].femur);
		dynamixel_writeword(3, AX_GOAL_POSITION_L, leg_angles[0].tibia);
		dynamixel_writeword(4, AX_GOAL_POSITION_L, leg_angles[0].tarsus);
		
		_delay_ms(100);
	}
	
	return 0;
}
