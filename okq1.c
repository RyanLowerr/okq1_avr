
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

	servo[0].direction  = FR_COXA_DIRECTION;
	servo[1].direction  = FR_FEMUR_DIRECTION;
	servo[2].direction  = FR_TIBIA_DIRECTION;
	servo[3].direction  = FR_TARSUS_DIRECTION;
	servo[4].direction  = BR_COXA_DIRECTION;
	servo[5].direction  = BR_FEMUR_DIRECTION;
	servo[6].direction  = BR_TIBIA_DIRECTION;
	servo[7].direction  = BR_TARSUS_DIRECTION;
	servo[8].direction  = BL_COXA_DIRECTION;
	servo[9].direction  = BL_FEMUR_DIRECTION;
	servo[10].direction = BL_TIBIA_DIRECTION;
	servo[11].direction = BL_TARSUS_DIRECTION;
	servo[12].direction = FL_COXA_DIRECTION;
	servo[13].direction = FL_FEMUR_DIRECTION;
	servo[14].direction = FL_TIBIA_DIRECTION;
	servo[15].direction = FL_TARSUS_DIRECTION;
	servo[16].direction = PAN_DIRECTION;
	servo[17].direction = TILT_DIRECTION;

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
		
		// ik calculations
		for(uint8_t i = 0; i < 4; i++)
		{
			ik_leg(x[i], y[i], z[i], &servo[i*4].angle, &servo[i*4+1].angle, &servo[i*4+2].angle, &servo[i*4+3].angle);
		}
		
		servo[16].angle = 0;
		servo[17].angle = 0;

		// form dynamixel sync write data
		n = 0;
		for(uint8_t i = 0; i < NUM_SERVOS; i++)
		{
			servo[i].position = (uint16_t) (AX_CENTER_VALUE + (servo[i].direction * (servo[i].angle + servo[i].center) * 3.41));
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
