#include <avr/interrupt.h>
#include <util/delay.h>

#include "types.h"
#include "common.h"
#include "position.h"
#include "motionmanager.h"
#include "joint.h"
#include "gait.h"
#include "controller.h"
#include "okirtrak.h"
#include "dynamixel.h"

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
	mm_init();
	
	while(1)
	{	
		mm_process(&controller, &gait);		
		//_delay_ms(1);
	}
	
	return 0;
}
