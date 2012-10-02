
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#include "dynamixel/dynamixel.h"
#include "dynamixel/ax.h"
#include "gait/gait.h"
#include "ik/ik.h"
#include "common.h"

void joint_init(jointdata* joint)
{
	joint[0].id  = FR_COXA_ID;
	joint[1].id  = FR_FEMUR_ID;
	joint[2].id  = FR_TIBIA_ID;
	joint[3].id  = FR_TARSUS_ID;
	joint[4].id  = BR_COXA_ID;
	joint[5].id  = BR_FEMUR_ID;
	joint[6].id  = BR_TIBIA_ID;
	joint[7].id  = BR_TARSUS_ID;
	joint[8].id  = BL_COXA_ID;
	joint[9].id  = BL_FEMUR_ID;
	joint[10].id = BL_TIBIA_ID;
	joint[11].id = BL_TARSUS_ID;
	joint[12].id = FL_COXA_ID;
	joint[13].id = FL_FEMUR_ID;
	joint[14].id = FL_TIBIA_ID;
	joint[15].id = FL_TARSUS_ID;
	joint[16].id = PAN_ID;
	joint[17].id = TILT_ID;

	joint[0].direction  = FR_COXA_DIRECTION;
	joint[1].direction  = FR_FEMUR_DIRECTION;
	joint[2].direction  = FR_TIBIA_DIRECTION;
	joint[3].direction  = FR_TARSUS_DIRECTION;
	joint[4].direction  = BR_COXA_DIRECTION;
	joint[5].direction  = BR_FEMUR_DIRECTION;
	joint[6].direction  = BR_TIBIA_DIRECTION;
	joint[7].direction  = BR_TARSUS_DIRECTION;
	joint[8].direction  = BL_COXA_DIRECTION;
	joint[9].direction  = BL_FEMUR_DIRECTION;
	joint[10].direction = BL_TIBIA_DIRECTION;
	joint[11].direction = BL_TARSUS_DIRECTION;
	joint[12].direction = FL_COXA_DIRECTION;
	joint[13].direction = FL_FEMUR_DIRECTION;
	joint[14].direction = FL_TIBIA_DIRECTION;
	joint[15].direction = FL_TARSUS_DIRECTION;
	joint[16].direction = PAN_DIRECTION;
	joint[17].direction = TILT_DIRECTION;

	joint[0].center  = -FR_COXA_ZERO;
	joint[1].center  =  FEMUR_ZERO;
	joint[2].center  = -TIBIA_ZERO;
	joint[3].center  =  TARSUS_ZERO;
	joint[4].center  =  BR_COXA_ZERO;
	joint[5].center  =  FEMUR_ZERO;
	joint[6].center  = -TIBIA_ZERO;
	joint[7].center  =  TARSUS_ZERO;
	joint[8].center  =  BL_COXA_ZERO;
	joint[9].center  =  FEMUR_ZERO;
	joint[10].center = -TIBIA_ZERO;
	joint[11].center =  TARSUS_ZERO;
	joint[12].center = -FL_COXA_ZERO;
	joint[13].center =  FEMUR_ZERO;
	joint[14].center = -TIBIA_ZERO;
	joint[15].center =  TARSUS_ZERO;
	joint[16].center =  PAN_ZERO;
	joint[17].center =  TILT_ZERO;
}

void foot_init(footdata* foot)
{
	foot[0].neutral_from_coxa_x = FOOT_X_NEUTRAL;
	foot[0].neutral_from_coxa_y = FOOT_Y_NEUTRAL;
	foot[0].neutral_from_center_x = FOOT_X_NEUTRAL + COXA_X_OFFSET;
	foot[0].neutral_from_center_y = FOOT_Y_NEUTRAL + COXA_Y_OFFSET;
	foot[0].neutral_z = -FOOT_Z_NEUTRAL;
	foot[0].coxa_offset_x = COXA_X_OFFSET;
	foot[0].coxa_offset_y = COXA_Y_OFFSET;

	foot[1].neutral_from_coxa_x = FOOT_X_NEUTRAL;
	foot[1].neutral_from_coxa_y = -FOOT_Y_NEUTRAL;
	foot[1].neutral_from_center_x = FOOT_X_NEUTRAL + COXA_X_OFFSET;
	foot[1].neutral_from_center_y = -FOOT_Y_NEUTRAL + -COXA_Y_OFFSET;
	foot[1].neutral_z = -FOOT_Z_NEUTRAL;
	foot[1].coxa_offset_x = COXA_X_OFFSET;
	foot[1].coxa_offset_y = -COXA_Y_OFFSET;
	
	foot[2].neutral_from_coxa_x = -FOOT_X_NEUTRAL;
	foot[2].neutral_from_coxa_y = -FOOT_Y_NEUTRAL;
	foot[2].neutral_from_center_x = -FOOT_X_NEUTRAL + -COXA_X_OFFSET;
	foot[2].neutral_from_center_y = -FOOT_Y_NEUTRAL + -COXA_Y_OFFSET;
	foot[2].neutral_z = -FOOT_Z_NEUTRAL;
	foot[2].coxa_offset_x = -COXA_X_OFFSET;
	foot[2].coxa_offset_y = -COXA_Y_OFFSET;
	
	foot[3].neutral_from_coxa_x = -FOOT_X_NEUTRAL;
	foot[3].neutral_from_coxa_y = FOOT_Y_NEUTRAL;
	foot[3].neutral_from_center_x = -FOOT_X_NEUTRAL + -COXA_X_OFFSET;
	foot[3].neutral_from_center_y = FOOT_Y_NEUTRAL + COXA_Y_OFFSET;
	foot[3].neutral_z = -FOOT_Z_NEUTRAL;
	foot[3].coxa_offset_x = -COXA_X_OFFSET;
	foot[3].coxa_offset_y = COXA_Y_OFFSET;
}

int main(void)
{
	gaitdata gait;
	jointdata joint[NUM_SERVOS];	
	footdata foot[4];
	float theta;
	uint8_t packet[54];
	uint8_t n;
	
	float controller_x = 0.0;
	float controller_y = 40.0;
 	float controller_z = 0.0;
	float controller_r = 20.0;
	float controller_s = 5.0;
		
	joint_init(&joint[0]);
	foot_init(&foot[0]);
	dynamixel_init();
	gait_init(&gait, GAIT_TYPE_RIPPLE);
	
	while(1)
	{	
		gait_process(&gait);
		
		if(controller_r == 0.0)
		{
			// Start with neutral foot position		
			for(int i = 0; i < 4; i++)
			{
				foot[i].x = foot[i].neutral_from_coxa_x;
				foot[i].y = foot[i].neutral_from_coxa_y;
				foot[i].z = foot[i].neutral_z;
			}
		}
		else
		{
			// start with rotations from neutral foot position
			for(int i = 0; i < 4; i++)
			{
				theta = gait.r[i] * controller_r * 0.01745;
				foot[i].x = (foot[i].neutral_from_center_x * cos(theta) - foot[i].neutral_from_center_y * sin(theta)) - foot[i].coxa_offset_x;
				foot[i].y = (foot[i].neutral_from_center_x * sin(theta) + foot[i].neutral_from_center_y * cos(theta)) - foot[i].coxa_offset_y;
				foot[i].z = foot[i].neutral_z;
			}
		}
		
		// Add gait translations to foot position
		for(int i = 0; i < 4; i++)
		{
			foot[i].x += gait.x[i] * controller_x;
			foot[i].y += gait.y[i] * controller_y;
			foot[i].z += gait.z[i] * controller_z;
		}
		
		// Add gait body shift if required.
		if(controller_s != 0.0)
		{
			gait_shift_process(&gait);
			for(int i = 0; i < 4; i++)
			{
				foot[i].x += gait.sx * controller_s;
				foot[i].y += gait.sy * controller_s;
			}
		}
		
		// Increment gait position
		gait_increment(&gait);
		
		// ik calculations
		for(uint8_t i = 0; i < 4; i++)
			ik_leg(foot[i].x, foot[i].y, foot[i].z, &joint[i*4].angle, &joint[i*4+1].angle, &joint[i*4+2].angle, &joint[i*4+3].angle);
		
		joint[16].angle = 0;
		joint[17].angle = 0;

		// form dynamixel sync write data
		n = 0;
		for(uint8_t i = 0; i < NUM_SERVOS; i++)
		{
			joint[i].position = (uint16_t) (AX_CENTER_VALUE + (joint[i].direction * (joint[i].angle + joint[i].center) * 3.41));
			packet[n++] = joint[i].id;
			packet[n++] = dynamixel_getlowbyte(joint[i].position);
			packet[n++] = dynamixel_gethighbyte(joint[i].position);
		}

		// write positions to servos
		dynamixel_syncwrite(AX_GOAL_POSITION_L, 2, NUM_SERVOS, &packet);	
		
		_delay_ms(5);
	}
	
	return 0;
}
