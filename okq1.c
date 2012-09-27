
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#include "dynamixel/dynamixel.h"
#include "dynamixel/ax.h"
#include "gait/gait.h"
#include "ik/ik.h"
#include "common.h"

int main(void)
{
	gaitdata gait;
	ik_angles leg_angles[4];	
	float x[4], y[4], z[4];
	uint8_t packet[48];

	dynamixel_init();
	gait_init(&gait, GAIT_TYPE_RIPPLE);
	
	while(1)
	{		
		gait_process(&gait);
		gait_increment(&gait);
				
		x[0] = FR_FOOT_X_NEUTRAL + gait.x[0] * 0.0;
		y[0] = FR_FOOT_Y_NEUTRAL + gait.y[0] * 50.0;
		z[0] = FR_FOOT_Z_NEUTRAL + gait.z[0] * 20.0;
		
		x[1] = BR_FOOT_X_NEUTRAL + gait.x[1] * 0.0;
		y[1] = BR_FOOT_Y_NEUTRAL + gait.y[1] * 50.0;
		z[1] = BR_FOOT_Z_NEUTRAL + gait.z[1] * 20.0;
		
		x[2] = BL_FOOT_X_NEUTRAL + gait.x[2] * 0.0;
		y[2] = BL_FOOT_Y_NEUTRAL + gait.y[2] * 50.0;
		z[2] = BL_FOOT_Z_NEUTRAL + gait.z[2] * 20.0;
		
		x[3] = FL_FOOT_X_NEUTRAL + gait.x[3] * 0.0;
		y[3] = FL_FOOT_Y_NEUTRAL + gait.y[3] * 50.0;
		z[3] = FL_FOOT_Z_NEUTRAL + gait.z[3] * 20.0;
		
		ik_leg(x[0], y[0], z[0], &leg_angles[0]);
		ik_leg(x[1], y[1], z[1], &leg_angles[1]);
		ik_leg(x[2], y[2], z[2], &leg_angles[2]);
		ik_leg(x[3], y[3], z[3], &leg_angles[3]);
	
		leg_angles[0].coxa   = (uint16_t) (AX_CENTER_VALUE + (leg_angles[0].coxa - 45.0) * 3.41);
		leg_angles[0].femur  = (uint16_t) (AX_CENTER_VALUE + (leg_angles[0].femur + 11.0) * 3.41);
		leg_angles[0].tibia  = (uint16_t) (AX_CENTER_VALUE - (leg_angles[0].tibia - 11.0) * 3.41);
		leg_angles[0].tarsus = (uint16_t) (AX_CENTER_VALUE - leg_angles[0].tarsus * 3.41);
		
		leg_angles[1].coxa   = (uint16_t) (AX_CENTER_VALUE + (leg_angles[1].coxa - 45.0) * 3.41);
		leg_angles[1].femur  = (uint16_t) (AX_CENTER_VALUE + (leg_angles[1].femur + 11.0) * 3.41);
		leg_angles[1].tibia  = (uint16_t) (AX_CENTER_VALUE - (leg_angles[1].tibia - 11.0) * 3.41);
		leg_angles[1].tarsus = (uint16_t) (AX_CENTER_VALUE - leg_angles[1].tarsus * 3.41);
		
		leg_angles[2].coxa   = (uint16_t) (AX_CENTER_VALUE + (leg_angles[2].coxa - 45.0) * 3.41);
		leg_angles[2].femur  = (uint16_t) (AX_CENTER_VALUE + (leg_angles[2].femur + 11.0) * 3.41);
		leg_angles[2].tibia  = (uint16_t) (AX_CENTER_VALUE - (leg_angles[2].tibia - 11.0) * 3.41);
		leg_angles[2].tarsus = (uint16_t) (AX_CENTER_VALUE - leg_angles[2].tarsus * 3.41);
		
		leg_angles[3].coxa   = (uint16_t) (AX_CENTER_VALUE + (leg_angles[3].coxa - 45.0) * 3.41);
		leg_angles[3].femur  = (uint16_t) (AX_CENTER_VALUE + (leg_angles[3].femur + 11.0) * 3.41);
		leg_angles[3].tibia  = (uint16_t) (AX_CENTER_VALUE - (leg_angles[3].tibia - 11.0) * 3.41);
		leg_angles[3].tarsus = (uint16_t) (AX_CENTER_VALUE - leg_angles[3].tarsus * 3.41);
		
		packet[0]  = 1;
		packet[1]  = dynamixel_getlowbyte(leg_angles[0].coxa);
		packet[2]  = dynamixel_gethighbyte(leg_angles[0].coxa);
		packet[3]  = 2;
		packet[4]  = dynamixel_getlowbyte(leg_angles[0].femur);
		packet[5]  = dynamixel_gethighbyte(leg_angles[0].femur);
		packet[6]  = 3;
		packet[7]  = dynamixel_getlowbyte(leg_angles[0].tibia);
		packet[8]  = dynamixel_gethighbyte(leg_angles[0].tibia);
		packet[9]  = 4;
		packet[10] = dynamixel_getlowbyte(leg_angles[0].tarsus);
		packet[11] = dynamixel_gethighbyte(leg_angles[0].tarsus);
		
		packet[12] = 5;
		packet[13] = dynamixel_getlowbyte(leg_angles[1].coxa);
		packet[14] = dynamixel_gethighbyte(leg_angles[1].coxa);
		packet[15] = 6;
		packet[16] = dynamixel_getlowbyte(leg_angles[1].femur);
		packet[17] = dynamixel_gethighbyte(leg_angles[1].femur);
		packet[18] = 7;
		packet[19] = dynamixel_getlowbyte(leg_angles[1].tibia);
		packet[20] = dynamixel_gethighbyte(leg_angles[1].tibia);
		packet[21] = 8;
		packet[22] = dynamixel_getlowbyte(leg_angles[1].tarsus);
		packet[23] = dynamixel_gethighbyte(leg_angles[1].tarsus);
		
		packet[24] = 9;
		packet[25] = dynamixel_getlowbyte(leg_angles[2].coxa);
		packet[26] = dynamixel_gethighbyte(leg_angles[2].coxa);
		packet[27] = 10;
		packet[28] = dynamixel_getlowbyte(leg_angles[2].femur);
		packet[29] = dynamixel_gethighbyte(leg_angles[2].femur);
		packet[30] = 11;
		packet[31] = dynamixel_getlowbyte(leg_angles[2].tibia);
		packet[32] = dynamixel_gethighbyte(leg_angles[2].tibia);
		packet[33] = 12;
		packet[34] = dynamixel_getlowbyte(leg_angles[2].tarsus);
		packet[35] = dynamixel_gethighbyte(leg_angles[2].tarsus);
		
		packet[36] = 13;
		packet[37] = dynamixel_getlowbyte(leg_angles[3].coxa);
		packet[38] = dynamixel_gethighbyte(leg_angles[3].coxa);
		packet[39] = 14;
		packet[40] = dynamixel_getlowbyte(leg_angles[3].femur);
		packet[41] = dynamixel_gethighbyte(leg_angles[3].femur);
		packet[42] = 15;
		packet[43] = dynamixel_getlowbyte(leg_angles[3].tibia);
		packet[44] = dynamixel_gethighbyte(leg_angles[3].tibia);
		packet[45] = 16;
		packet[46] = dynamixel_getlowbyte(leg_angles[3].tarsus);
		packet[47] = dynamixel_gethighbyte(leg_angles[3].tarsus);
		
		dynamixel_syncwrite(AX_GOAL_POSITION_L, 2, 16, &packet);
		
		_delay_ms(10);
	}
	
	return 0;
}
