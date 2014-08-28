
#include <avr/interrupt.h>

#include "types.h"
#include "common.h"
#include "position.h"
#include "motion.h"
#include "gait.h"
#include "controller.h"
#include "dynamixel.h"

int main(void)
{
	// Enable global interups.
	sei();
	
	// Perform initizations
	servo[ 0] = dynamixel_new(DYNAMIXEL_TYPE_MX, FR_COXA_ID,   FR_COXA_DIRECTION,   FR_COXA_ZERO);
	servo[ 1] = dynamixel_new(DYNAMIXEL_TYPE_MX, FR_FEMUR_ID,  FR_FEMUR_DIRECTION,  FEMUR_ZERO);
	servo[ 2] = dynamixel_new(DYNAMIXEL_TYPE_MX, FR_TIBIA_ID,  FR_TIBIA_DIRECTION,  TIBIA_ZERO);
	servo[ 3] = dynamixel_new(DYNAMIXEL_TYPE_MX, FR_TARSUS_ID, FR_TARSUS_DIRECTION, TARSUS_ZERO);
	
	servo[ 4] = dynamixel_new(DYNAMIXEL_TYPE_MX, BR_COXA_ID,   BR_COXA_DIRECTION,   BR_COXA_ZERO);
	servo[ 5] = dynamixel_new(DYNAMIXEL_TYPE_MX, BR_FEMUR_ID,  BR_FEMUR_DIRECTION,  FEMUR_ZERO);
	servo[ 6] = dynamixel_new(DYNAMIXEL_TYPE_MX, BR_TIBIA_ID,  BR_TIBIA_DIRECTION,  TIBIA_ZERO);
	servo[ 7] = dynamixel_new(DYNAMIXEL_TYPE_MX, BR_TARSUS_ID, BR_TARSUS_DIRECTION, TARSUS_ZERO);
	
	servo[ 8] = dynamixel_new(DYNAMIXEL_TYPE_MX, BL_COXA_ID,   BL_COXA_DIRECTION,   BL_COXA_ZERO);
	servo[ 9] = dynamixel_new(DYNAMIXEL_TYPE_MX, BL_FEMUR_ID,  BL_FEMUR_DIRECTION,  FEMUR_ZERO);
	servo[10] = dynamixel_new(DYNAMIXEL_TYPE_MX, BL_TIBIA_ID,  BL_TIBIA_DIRECTION,  TIBIA_ZERO);
	servo[11] = dynamixel_new(DYNAMIXEL_TYPE_MX, BL_TARSUS_ID, BL_TARSUS_DIRECTION, TARSUS_ZERO);
	
	servo[12] = dynamixel_new(DYNAMIXEL_TYPE_MX, FL_COXA_ID,   FL_COXA_DIRECTION,   FL_COXA_ZERO);
	servo[13] = dynamixel_new(DYNAMIXEL_TYPE_MX, FL_FEMUR_ID,  FL_FEMUR_DIRECTION,  FEMUR_ZERO);
	servo[14] = dynamixel_new(DYNAMIXEL_TYPE_MX, FL_TIBIA_ID,  FL_TIBIA_DIRECTION,  TIBIA_ZERO);
	servo[15] = dynamixel_new(DYNAMIXEL_TYPE_MX, FL_TARSUS_ID, FL_TARSUS_DIRECTION, TARSUS_ZERO);
	
	gait_init(&gait, GAIT_TYPE_AMBLE);
	
	controller_init(&controller);
	
	motion_init();
	
	dynamixel_init();
	
	position_init();
	
	// Main program loop.
	while(1)
	{
		motion_process(&controller);
		dynamixel_write_positions(&servo[0]);
	}
	
	return 0;
}
