
#include <avr/io.h>

#include "dynamixel/dynamixel.h"
#include "dynamixel/ax.h"
#include "ik/ik.h"

int main(void)
{
	ik_angles leg_angles;
	
	ik_leg(70.0, 70.0, -20.0, &leg_angles);
	
	leg_angles.coxa   = (int) (AX_CENTER_VALUE + leg_angles.coxa * 3.41);
	leg_angles.femur  = (int) (AX_CENTER_VALUE + (leg_angles.femur + 11.0) * 3.41);
	leg_angles.tibia  = (int) (AX_CENTER_VALUE + (leg_angles.tibia - 11.0) * 3.41);
	leg_angles.tarsus = (int) (AX_CENTER_VALUE + leg_angles.tarsus * 3.41);
	
	dynamixel_init();
	dynamixel_writeword(1, AX_GOAL_POSITION_L, leg_angles.coxa);
	dynamixel_writeword(2, AX_GOAL_POSITION_L, leg_angles.femur);
	dynamixel_writeword(3, AX_GOAL_POSITION_L, leg_angles.tibia);
	dynamixel_writeword(4, AX_GOAL_POSITION_L, leg_angles.tarsus);
	
	return 0;
}
