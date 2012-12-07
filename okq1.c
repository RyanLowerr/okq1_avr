
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#include "dynamixel.h"
#include "mx.h"
#include "controller.h"
#include "gait.h"
#include "ik.h"
#include "common.h"
#include "twi.h"
#include "camera.h"

// Save joint constants into joint structure from common.h file
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
	joint[16].id = TURRET_PAN_ID;
	joint[17].id = TURRET_TILT_ID;
	joint[18].id = R_GUN_PAN_ID;
	joint[19].id = R_GUN_TILT_ID;
	joint[20].id = L_GUN_PAN_ID;
	joint[21].id = L_GUN_TILT_ID;

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
	joint[16].direction = TURRET_PAN_DIRECTION;
	joint[17].direction = TURRET_TILT_DIRECTION;
	joint[18].direction = R_GUN_PAN_DIRECTION;
	joint[19].direction = R_GUN_TILT_DIRECTION;
	joint[20].direction = L_GUN_PAN_DIRECTION;
	joint[21].direction = L_GUN_TILT_DIRECTION;

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
	joint[16].center =  TURRET_PAN_ZERO;
	joint[17].center =  TURRET_TILT_ZERO;
	joint[18].center =  R_GUN_PAN_ZERO;
	joint[19].center =  R_GUN_TILT_ZERO;
	joint[20].center =  L_GUN_PAN_ZERO;
	joint[21].center =  L_GUN_TILT_ZERO;
}

// Write joint positions out to all servos using the dynamixel sync write command
void joint_write(jointdata* joint)
{
	uint8_t packet[NUM_SERVOS*3]; // id, position low byte and position high byte per servo
	uint8_t n = 0;
	
	for(uint8_t i = 0; i < NUM_SERVOS; i++)
	{
		joint[i].position = (uint16_t) (MX_CENTER_VALUE + (joint[i].direction * (joint[i].angle + joint[i].center) * MX_TIC_PER_DEG));
		packet[n++] = joint[i].id;
		packet[n++] = dynamixel_getlowbyte(joint[i].position);
		packet[n++] = dynamixel_gethighbyte(joint[i].position);
	}

	dynamixel_syncwrite(MX_GOAL_POSITION_L, 2, NUM_SERVOS, &packet);
}

// Calculate and save foot neutral positions and offset values into foot structure
void foot_init(footdata* foot)
{
	foot[0].neutral_from_coxa_x   =  FOOT_X_NEUTRAL;
	foot[0].neutral_from_coxa_y   =  FOOT_Y_NEUTRAL;
	foot[0].neutral_from_center_x =  FOOT_X_NEUTRAL + COXA_X_OFFSET;
	foot[0].neutral_from_center_y =  FOOT_Y_NEUTRAL + COXA_Y_OFFSET;
	foot[0].neutral_z             = -FOOT_Z_NEUTRAL;
	foot[0].coxa_offset_x         =  COXA_X_OFFSET;
	foot[0].coxa_offset_y         =  COXA_Y_OFFSET;

	foot[1].neutral_from_coxa_x   =  FOOT_X_NEUTRAL;
	foot[1].neutral_from_coxa_y   = -FOOT_Y_NEUTRAL;
	foot[1].neutral_from_center_x =  FOOT_X_NEUTRAL + COXA_X_OFFSET;
	foot[1].neutral_from_center_y = -FOOT_Y_NEUTRAL - COXA_Y_OFFSET;
	foot[1].neutral_z             = -FOOT_Z_NEUTRAL;
	foot[1].coxa_offset_x         =  COXA_X_OFFSET;
	foot[1].coxa_offset_y         = -COXA_Y_OFFSET;
	
	foot[2].neutral_from_coxa_x   = -FOOT_X_NEUTRAL;
	foot[2].neutral_from_coxa_y   = -FOOT_Y_NEUTRAL;
	foot[2].neutral_from_center_x = -FOOT_X_NEUTRAL - COXA_X_OFFSET;
	foot[2].neutral_from_center_y = -FOOT_Y_NEUTRAL - COXA_Y_OFFSET;
	foot[2].neutral_z             = -FOOT_Z_NEUTRAL;
	foot[2].coxa_offset_x         = -COXA_X_OFFSET;
	foot[2].coxa_offset_y         = -COXA_Y_OFFSET;
	
	foot[3].neutral_from_coxa_x   = -FOOT_X_NEUTRAL;
	foot[3].neutral_from_coxa_y   =  FOOT_Y_NEUTRAL;
	foot[3].neutral_from_center_x = -FOOT_X_NEUTRAL - COXA_X_OFFSET;
	foot[3].neutral_from_center_y =  FOOT_Y_NEUTRAL + COXA_Y_OFFSET;
	foot[3].neutral_z             = -FOOT_Z_NEUTRAL;
	foot[3].coxa_offset_x         = -COXA_X_OFFSET;
	foot[3].coxa_offset_y         =  COXA_Y_OFFSET;
}

// Calculate x/y/z foot positions from controller and gait data
void foot_position_calc(footdata* foot, controllerdata* controller, gaitdata* gait)
{
	if(controller->r == 0.0)
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
		float theta;
		float costheta;
		float sintheta;
		
		for(int i = 0; i < 4; i++)
		{
			theta = gait->r[i] * controller->r * 0.01745;
			costheta = cos(theta);
			sintheta = sin(theta);
				
			foot[i].x = (foot[i].neutral_from_center_x * costheta - foot[i].neutral_from_center_y * sintheta) - foot[i].coxa_offset_x;
			foot[i].y = (foot[i].neutral_from_center_x * sintheta + foot[i].neutral_from_center_y * costheta) - foot[i].coxa_offset_y;
			foot[i].z = foot[i].neutral_z;
		}
	}
		
	// Add gait translations to foot position
	for(int i = 0; i < 4; i++)
	{
		foot[i].x += gait->x[i] * controller->x;
		foot[i].y += gait->y[i] * controller->y;
		foot[i].z += gait->z[i] * controller->z;
	}
		
	// Add gait body shift if required.
	if(controller->s != 0.0)
	{
		gait_shift_process(gait);
		for(int i = 0; i < 4; i++)
		{
			foot[i].x -= gait->sx * controller->s;
			foot[i].y -= gait->sy * controller->s;
		}
	}
}

int main(void)
{
	jointdata joint[NUM_SERVOS];
	joint_init(&joint[0]);
	
	footdata foot[4];
	foot_init(&foot[0]);
	
	controllerdata controller;
	controller_init();
	controller.x = 0.0;
	controller.y = 30.0;
 	controller.z = 40.0;
	controller.r = 0.0;
	controller.s = 0.0;
	
	gaitdata gait;
	gait_init(&gait, GAIT_TYPE_AMBLE);
	gait_process(&gait);
	
	dynamixel_init();
	
	twi_init();
	
	while(1)
	{	
		gait_process(&gait);
		foot_position_calc(&foot[0], &controller, &gait);
		gait_increment(&gait);
		
		for(uint8_t i = 0; i < 4; i++)
			ik_leg(foot[i].x, foot[i].y, foot[i].z, &joint[i*4].angle, &joint[i*4+1].angle, &joint[i*4+2].angle, &joint[i*4+3].angle);
			
		joint[16].angle = 0;
		joint[17].angle = 0;

		joint_write(&joint[0]);
		
		_delay_ms(5);
	}
	
	return 0;
}
