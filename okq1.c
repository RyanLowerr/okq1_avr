
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "common.h"
#include "position.h"
#include "motion.h"
#include "joint.h"
#include "gait.h"
#include "controller.h"
#include "camera.h"
#include "position.h"

int main(void)
{
	// enable interups
	sei();

	// initalize datasets that store joint values
	joint_init(&joint[0]);
	
	// initalize datasets that store gait values
	gait_init(&gait, GAIT_TYPE_AMBLE);
	gait_process(&gait);
	
	// initalize controller
	controller_init(&controller);
	
	// initalize dynamixel bus
	dynamixel_init();

	// initalize robot positions
	position_init();

	// initalize the motion manager
	motion_init();
	
	while(1)
	{	
		motion_process(&controller, &gait);		
		_delay_ms(1);
	}
	
	return 0;
}
