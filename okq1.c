
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#include "dynamixel/dynamixel.h"
#include "dynamixel/ax.h"
#include "ik/ik.h"

int main(void)
{
	ik_angles leg_angles;
	
	int angle = 0;
	int cx = 85;
	int cy = 85;
	int r = 25;
	
	double x;
	double y;
	
	dynamixel_init();
	
	while(1)
	{
		x = cx + r * cos(angle * 0.0174);
		y = cy + r * sin(angle * 0.0174);
	
		ik_leg(x, y, -30.0, &leg_angles);
	
		leg_angles.coxa   = (int) (AX_CENTER_VALUE + (leg_angles.coxa - 45.0) * 3.41);
		leg_angles.femur  = (int) (AX_CENTER_VALUE + (leg_angles.femur + 11.0) * 3.41);
		leg_angles.tibia  = (int) (AX_CENTER_VALUE - (leg_angles.tibia - 11.0) * 3.41);
		leg_angles.tarsus = (int) (AX_CENTER_VALUE - leg_angles.tarsus * 3.41);
	
		dynamixel_writeword(1, AX_GOAL_POSITION_L, leg_angles.coxa);
		dynamixel_writeword(2, AX_GOAL_POSITION_L, leg_angles.femur);
		dynamixel_writeword(3, AX_GOAL_POSITION_L, leg_angles.tibia);
		dynamixel_writeword(4, AX_GOAL_POSITION_L, leg_angles.tarsus);
	
		angle = angle + 5;
		if(angle > 360) 
			angle = 0;
	}
	
	return 0;
}
