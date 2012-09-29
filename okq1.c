
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#include "dynamixel/dynamixel.h"
#include "dynamixel/ax.h"
#include "gait/gait.h"
#include "ik/ik.h"
#include "common.h"

void servo_init(dynamixel* servo)
{
	servo[0].id  = FR_COXA_ID;
	servo[1].id  = FR_FEMUR_ID;
	servo[2].id  = FR_TIBIA_ID;
	servo[3].id  = FR_TARSUS_ID;
	servo[4].id  = BR_COXA_ID;
	servo[5].id  = BR_FEMUR_ID;
	servo[6].id  = BR_TIBIA_ID;
	servo[7].id  = BR_TARSUS_ID;
	servo[8].id  = BL_COXA_ID;
	servo[9].id  = BL_FEMUR_ID;
	servo[10].id = BL_TIBIA_ID;
	servo[11].id = BL_TARSUS_ID;
	servo[12].id = FL_COXA_ID;
	servo[13].id = FL_FEMUR_ID;
	servo[14].id = FL_TIBIA_ID;
	servo[15].id = FL_TARSUS_ID;
	servo[16].id = PAN_ID;
	servo[17].id = TILT_ID;

	servo[0].direction  =  1.0;
	servo[1].direction  =  1.0;
	servo[2].direction  = -1.0;
	servo[3].direction  = -1.0;
	servo[4].direction  =  1.0;
	servo[5].direction  = -1.0;
	servo[6].direction  =  1.0;
	servo[7].direction  =  1.0;
	servo[8].direction  =  1.0;
	servo[9].direction  =  1.0;
	servo[10].direction = -1.0;
	servo[11].direction = -1.0;
	servo[12].direction =  1.0;
	servo[13].direction = -1.0;
	servo[14].direction =  1.0;
	servo[15].direction =  1.0;
	servo[16].direction =  1.0;
	servo[17].direction =  1.0;

	servo[0].center  = -FR_COXA_ZERO;
	servo[1].center  =  FEMUR_ZERO;
	servo[2].center  = -TIBIA_ZERO;
	servo[3].center  =  TARSUS_ZERO;
	servo[4].center  =  BR_COXA_ZERO;
	servo[5].center  =  FEMUR_ZERO;
	servo[6].center  = -TIBIA_ZERO;
	servo[7].center  =  TARSUS_ZERO;
	servo[8].center  =  BL_COXA_ZERO;
	servo[9].center  =  FEMUR_ZERO;
	servo[10].center = -TIBIA_ZERO;
	servo[11].center =  TARSUS_ZERO;
	servo[12].center = -FL_COXA_ZERO;
	servo[13].center =  FEMUR_ZERO;
	servo[14].center = -TIBIA_ZERO;
	servo[15].center =  TARSUS_ZERO;
	servo[16].center =  PAN_ZERO;
	servo[17].center =  TILT_ZERO;
}

int main(void)
{
	gaitdata gait;
	dynamixel servo[NUM_SERVOS];
	ikdata ikresults[4];	
	float x[4], y[4], z[4];
	uint8_t packet[54];
	uint8_t n;
	
	servo_init(&servo[0]);
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
		servo[0].angle = ikresults[0].coxa;
		servo[1].angle = ikresults[0].femur;
		servo[2].angle = ikresults[0].tibia;
		servo[3].angle = ikresults[0].tarsus;
		
		ik_leg(x[1], y[1], z[1], &ikresults[1]);
		servo[4].angle = ikresults[1].coxa;
		servo[5].angle = ikresults[1].femur;
		servo[6].angle = ikresults[1].tibia;
		servo[7].angle = ikresults[1].tarsus;
		
		ik_leg(x[2], y[2], z[2], &ikresults[2]);
		servo[8].angle = ikresults[2].coxa;
		servo[9].angle = ikresults[2].femur;
		servo[10].angle = ikresults[2].tibia;
		servo[11].angle = ikresults[2].tarsus;
		
		ik_leg(x[3], y[3], z[3], &ikresults[3]);
		servo[12].angle = ikresults[3].coxa;
		servo[13].angle = ikresults[3].femur;
		servo[14].angle = ikresults[3].tibia;
		servo[15].angle = ikresults[3].tarsus;
		
		servo[17].angle = 0;
		servo[18].angle = 0;
		
		// calculate servo positions
		for(uint8_t i = 0; i < NUM_SERVOS; i++)
		{
			servo[i].position = (uint16_t) (AX_CENTER_VALUE + (servo[i].direction * (servo[i].angle + servo[i].center) * 3.41));
		}

		// form dynamixel sync write data
		n = 0;
		for(uint8_t i = 0; i < NUM_SERVOS; i++)
		{
			packet[n++] = servo[i].id;
			packet[n++] = dynamixel_getlowbyte(servo[i].position);
			packet[n++] = dynamixel_gethighbyte(servo[i].position);
		}

		// write positions to servos
		dynamixel_syncwrite(AX_GOAL_POSITION_L, 2, NUM_SERVOS, &packet);	
		
		_delay_ms(10);
	}
	
	return 0;
}
