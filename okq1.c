
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#include "dynamixel/dynamixel.h"
#include "dynamixel/ax.h"
#include "gait/gait.h"
#include "ik/ik.h"

int main(void)
{
	ik_angles leg_angles[4];
	gaitdata gait;	
	double x, y, z;
	uint8_t packet[128];

	dynamixel_init();
	gait_init(&gait, GAIT_TYPE_RIPPLE);
	
	while(1)
	{		
		gait_process(&gait);
		
		x = 40.0 + gait.x[0] * 50.0;
		y = 100.0 + gait.y[0] * 0.0;
		z = -30.0 + gait.z[0] * 30.0;
		
		ik_leg(x, y, z, &leg_angles[0]);
	
		leg_angles[0].coxa   = (int) (AX_CENTER_VALUE + (leg_angles[0].coxa - 45.0) * 3.41);
		leg_angles[0].femur  = (int) (AX_CENTER_VALUE + (leg_angles[0].femur + 11.0) * 3.41);
		leg_angles[0].tibia  = (int) (AX_CENTER_VALUE - (leg_angles[0].tibia - 11.0) * 3.41);
		leg_angles[0].tarsus = (int) (AX_CENTER_VALUE - leg_angles[0].tarsus * 3.41);
	
		packet[0] = 1;
		packet[1] = dynamixel_getlowbyte(leg_angles[0].coxa);
		packet[2] = dynamixel_gethighbyte(leg_angles[0].coxa);
		packet[3] = 2;
		packet[4] = dynamixel_getlowbyte(leg_angles[0].femur);
		packet[5] = dynamixel_gethighbyte(leg_angles[0].femur);
		packet[6] = 3;
		packet[7] = dynamixel_getlowbyte(leg_angles[0].tibia);
		packet[8] = dynamixel_gethighbyte(leg_angles[0].tibia);
		packet[9] = 4;
		packet[10] = dynamixel_getlowbyte(leg_angles[0].tarsus);
		packet[11] = dynamixel_gethighbyte(leg_angles[0].tarsus);
	
		dynamixel_syncwrite(AX_GOAL_POSITION_L, 2, 4, &packet);
		
		_delay_ms(10);
	}
	
	return 0;
}
