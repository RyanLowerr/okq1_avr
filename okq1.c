
#include <avr/interrupt.h>

#include "types.h"
#include "common.h"
#include "position.h"
#include "motion.h"
#include "joint.h"
#include "gait.h"
#include "controller.h"
#include "okirtrak.h"
#include "dynamixel.h"

int main(void)
{
	// Enable global interups.
	sei();
	
	// Perform initizations.
	joint_init(&joint[0]);
	gait_init(&gait, GAIT_TYPE_AMBLE);
	controller_init(&controller);
	motion_init(&motion);
	dynamixel_init();
	position_init();
	
	// Main program loop.
	while(1)
	{
		motion_process(&motion, &controller);
		joint_process(&joint[0]);
	}
	
	return 0;
}
