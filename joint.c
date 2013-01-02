
#include <avr/io.h>

#include "joint.h"
#include "common.h"
#include "dynamixel.h"
#include "ax.h"
#include "mx.h"

JOINT joint[NUM_SERVOS];

// Save joint constants into joint structure from common.h file
void joint_init(JOINT *joint)
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

	joint[0].center  = FR_COXA_ZERO;
	joint[1].center  = FEMUR_ZERO;
	joint[2].center  = TIBIA_ZERO;
	joint[3].center  = TARSUS_ZERO;
	
	joint[4].center  = BR_COXA_ZERO;
	joint[5].center  = FEMUR_ZERO;
	joint[6].center  = TIBIA_ZERO;
	joint[7].center  = TARSUS_ZERO;
	
	joint[8].center  = BL_COXA_ZERO;
	joint[9].center  = FEMUR_ZERO;
	joint[10].center = TIBIA_ZERO;
	joint[11].center = TARSUS_ZERO;
	
	joint[12].center = FL_COXA_ZERO;
	joint[13].center = FEMUR_ZERO;
	joint[14].center = TIBIA_ZERO;
	joint[15].center = TARSUS_ZERO;
	
	joint[16].center = TURRET_PAN_ZERO;
	joint[17].center = TURRET_TILT_ZERO;
	
	joint[18].center = R_GUN_PAN_ZERO;
	joint[19].center = R_GUN_TILT_ZERO;
	joint[20].center = L_GUN_PAN_ZERO;
	joint[21].center = L_GUN_TILT_ZERO;

	joint[0].type  = JOINT_TYPE_MX;
	joint[1].type  = JOINT_TYPE_MX;
	joint[2].type  = JOINT_TYPE_MX;
	joint[3].type  = JOINT_TYPE_MX;
	joint[4].type  = JOINT_TYPE_MX;
	joint[5].type  = JOINT_TYPE_MX;
	joint[6].type  = JOINT_TYPE_MX;
	joint[7].type  = JOINT_TYPE_MX;
	joint[8].type  = JOINT_TYPE_MX;
	joint[9].type  = JOINT_TYPE_MX;
	joint[10].type = JOINT_TYPE_MX;
	joint[11].type = JOINT_TYPE_MX;
	joint[12].type = JOINT_TYPE_MX;
	joint[13].type = JOINT_TYPE_MX;
	joint[14].type = JOINT_TYPE_MX;
	joint[15].type = JOINT_TYPE_MX;
	joint[16].type = JOINT_TYPE_MX;
	joint[17].type = JOINT_TYPE_MX;
	joint[18].type = JOINT_TYPE_AX;
	joint[19].type = JOINT_TYPE_AX;
	joint[20].type = JOINT_TYPE_AX;
	joint[21].type = JOINT_TYPE_AX;
}

// Writes joint positions to all servos that need their goal positions updated using the dynamixel sync write command
void joint_write(JOINT *joint)
{
	uint8_t packet[NUM_SERVOS*3]; // id, position low byte and position high byte per servo
	uint8_t servocount = 0;       // count of servos that need position update
	uint8_t n = 0;
	
	for(uint8_t i = 0; i < NUM_SERVOS; i++)
	{	
		// Calculate the joint's position.
		if(joint[i].type == JOINT_TYPE_MX)
			joint[i].position = (uint16_t) (MX_CENTER_VALUE + (MX_TIC_PER_DEG * (joint[i].direction * (joint[i].center + joint[i].angle))));
		else if (joint[i].type == JOINT_TYPE_AX)
			joint[i].position = (uint16_t) (AX_CENTER_VALUE + (AX_TIC_PER_DEG * (joint[i].direction * (joint[i].center + joint[i].angle))));
		else
			joint[i].position = 0;

		// If the joint requires a position update add it to the sync write packet. Increment the servo counter.		
		if(joint[i].position != joint[i].prevposition) 
		{
			packet[n++] = joint[i].id;
			packet[n++] = dynamixel_getlowbyte(joint[i].position);
			packet[n++] = dynamixel_gethighbyte(joint[i].position);
			servocount++;
		}

		// Remember the joint's newest position value.
		joint[i].prevposition = joint[i].position;
	}

	// sync write goal positions out to servos if required.
	if(servocount > 0)
	 dynamixel_syncwrite(MX_GOAL_POSITION_L, 2, servocount, &packet);
}
