
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
	ikdata ikresults[4];	
	float x[4], y[4], z[4];
	uint8_t packet[54];

	dynamixel_init();
	gait_init(&gait, GAIT_TYPE_RIPPLE);
	
	while(1)
	{		
		gait_process(&gait);
		gait_increment(&gait);
				
		x[0] = FR_FOOT_X_NEUTRAL + gait.x[0] * 0.0;
		y[0] = FR_FOOT_Y_NEUTRAL + gait.y[0] * 40.0;
		z[0] = FR_FOOT_Z_NEUTRAL + gait.z[0] * 30.0;
		
		x[1] = BR_FOOT_X_NEUTRAL + gait.x[1] * 0.0;
		y[1] = BR_FOOT_Y_NEUTRAL + gait.y[1] * 40.0;
		z[1] = BR_FOOT_Z_NEUTRAL + gait.z[1] * 30.0;
		
		x[2] = BL_FOOT_X_NEUTRAL + gait.x[2] * 0.0;
		y[2] = BL_FOOT_Y_NEUTRAL + gait.y[2] * 40.0;
		z[2] = BL_FOOT_Z_NEUTRAL + gait.z[2] * 30.0;
		
		x[3] = FL_FOOT_X_NEUTRAL + gait.x[3] * 0.0;
		y[3] = FL_FOOT_Y_NEUTRAL + gait.y[3] * 40.0;
		z[3] = FL_FOOT_Z_NEUTRAL + gait.z[3] * 30.0;
		
		ik_leg(x[0], y[0], z[0], &ikresults[0]);
		ik_leg(x[1], y[1], z[1], &ikresults[1]);
		ik_leg(x[2], y[2], z[2], &ikresults[2]);
		ik_leg(x[3], y[3], z[3], &ikresults[3]);
	
		ikresults[0].coxa   = (uint16_t) (AX_CENTER_VALUE + (ikresults[0].coxa - FR_COXA_ZERO) * 3.41);
		ikresults[1].coxa   = (uint16_t) (AX_CENTER_VALUE + (ikresults[1].coxa + BR_COXA_ZERO) * 3.41);
		ikresults[2].coxa   = (uint16_t) (AX_CENTER_VALUE + (ikresults[2].coxa + BL_COXA_ZERO) * 3.41);
		ikresults[3].coxa   = (uint16_t) (AX_CENTER_VALUE + (ikresults[3].coxa - FL_COXA_ZERO) * 3.41);
				
		ikresults[0].femur  = (uint16_t) (AX_CENTER_VALUE + (ikresults[0].femur + FEMUR_ZERO) * 3.41);
		ikresults[1].femur  = (uint16_t) (AX_CENTER_VALUE - (ikresults[1].femur + FEMUR_ZERO) * 3.41);
		ikresults[2].femur  = (uint16_t) (AX_CENTER_VALUE + (ikresults[2].femur + FEMUR_ZERO) * 3.41);
		ikresults[3].femur  = (uint16_t) (AX_CENTER_VALUE - (ikresults[3].femur + FEMUR_ZERO) * 3.41);
		
		ikresults[0].tibia  = (uint16_t) (AX_CENTER_VALUE - (ikresults[0].tibia - TIBIA_ZERO) * 3.41);
		ikresults[1].tibia  = (uint16_t) (AX_CENTER_VALUE + (ikresults[1].tibia - TIBIA_ZERO) * 3.41);
		ikresults[2].tibia  = (uint16_t) (AX_CENTER_VALUE - (ikresults[2].tibia - TIBIA_ZERO) * 3.41);
		ikresults[3].tibia  = (uint16_t) (AX_CENTER_VALUE + (ikresults[3].tibia - TIBIA_ZERO) * 3.41);
		
		ikresults[0].tarsus = (uint16_t) (AX_CENTER_VALUE - (ikresults[0].tarsus + TARSUS_ZERO) * 3.41);
		ikresults[1].tarsus = (uint16_t) (AX_CENTER_VALUE + (ikresults[1].tarsus + TARSUS_ZERO) * 3.41);
		ikresults[2].tarsus = (uint16_t) (AX_CENTER_VALUE - (ikresults[2].tarsus + TARSUS_ZERO) * 3.41);
		ikresults[3].tarsus = (uint16_t) (AX_CENTER_VALUE + (ikresults[3].tarsus + TARSUS_ZERO) * 3.41);
		
		packet[0]  = FR_COXA_ID;
		packet[1]  = dynamixel_getlowbyte(ikresults[0].coxa);
		packet[2]  = dynamixel_gethighbyte(ikresults[0].coxa);
		packet[3]  = FR_FEMUR_ID;
		packet[4]  = dynamixel_getlowbyte(ikresults[0].femur);
		packet[5]  = dynamixel_gethighbyte(ikresults[0].femur);
		packet[6]  = FR_TIBIA_ID;
		packet[7]  = dynamixel_getlowbyte(ikresults[0].tibia);
		packet[8]  = dynamixel_gethighbyte(ikresults[0].tibia);
		packet[9]  = FR_TARSUS_ID;
		packet[10] = dynamixel_getlowbyte(ikresults[0].tarsus);
		packet[11] = dynamixel_gethighbyte(ikresults[0].tarsus);
		
		packet[12] = BR_COXA_ID;
		packet[13] = dynamixel_getlowbyte(ikresults[1].coxa);
		packet[14] = dynamixel_gethighbyte(ikresults[1].coxa);
		packet[15] = BR_FEMUR_ID;
		packet[16] = dynamixel_getlowbyte(ikresults[1].femur);
		packet[17] = dynamixel_gethighbyte(ikresults[1].femur);
		packet[18] = BR_TIBIA_ID;
		packet[19] = dynamixel_getlowbyte(ikresults[1].tibia);
		packet[20] = dynamixel_gethighbyte(ikresults[1].tibia);
		packet[21] = BR_TARSUS_ID;
		packet[22] = dynamixel_getlowbyte(ikresults[1].tarsus);
		packet[23] = dynamixel_gethighbyte(ikresults[1].tarsus);
		
		packet[24] = BL_COXA_ID;
		packet[25] = dynamixel_getlowbyte(ikresults[2].coxa);
		packet[26] = dynamixel_gethighbyte(ikresults[2].coxa);
		packet[27] = BL_FEMUR_ID;
		packet[28] = dynamixel_getlowbyte(ikresults[2].femur);
		packet[29] = dynamixel_gethighbyte(ikresults[2].femur);
		packet[30] = BL_TIBIA_ID;
		packet[31] = dynamixel_getlowbyte(ikresults[2].tibia);
		packet[32] = dynamixel_gethighbyte(ikresults[2].tibia);
		packet[33] = BL_TARSUS_ID;
		packet[34] = dynamixel_getlowbyte(ikresults[2].tarsus);
		packet[35] = dynamixel_gethighbyte(ikresults[2].tarsus);
		
		packet[36] = FL_COXA_ID;
		packet[37] = dynamixel_getlowbyte(ikresults[3].coxa);
		packet[38] = dynamixel_gethighbyte(ikresults[3].coxa);
		packet[39] = FL_FEMUR_ID;
		packet[40] = dynamixel_getlowbyte(ikresults[3].femur);
		packet[41] = dynamixel_gethighbyte(ikresults[3].femur);
		packet[42] = FL_TIBIA_ID;
		packet[43] = dynamixel_getlowbyte(ikresults[3].tibia);
		packet[44] = dynamixel_gethighbyte(ikresults[3].tibia);
		packet[45] = FL_TARSUS_ID;
		packet[46] = dynamixel_getlowbyte(ikresults[3].tarsus);
		packet[47] = dynamixel_gethighbyte(ikresults[3].tarsus);
		
		packet[48] = PAN_ID;
		packet[49] = dynamixel_getlowbyte(AX_CENTER_VALUE);
		packet[50] = dynamixel_gethighbyte(AX_CENTER_VALUE);
		packet[51] = TILT_ID;
		packet[52] = dynamixel_getlowbyte(AX_CENTER_VALUE);
		packet[53] = dynamixel_gethighbyte(AX_CENTER_VALUE);
				
		dynamixel_syncwrite(AX_GOAL_POSITION_L, 2, 18, &packet);
		
		_delay_ms(10);
	}
	
	return 0;
}
