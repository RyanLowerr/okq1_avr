
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#include "common.h"
#include "foot.h"
#include "joint.h"
#include "kinematics.h"
#include "gait.h"
#include "controller.h"
#include "dynamixel.h"
#include "mx.h"
#include "camera.h"

int main(void)
{
	// enable interups
	sei();

	// initalize datasets to store foot placement values
	footdata foot[4];
	foot_init(&foot[0]);

	// initalize datasets to store joint values
	jointdata joint[NUM_SERVOS];
	joint_init(&joint[0]);
	
	// initalize datasets to store gait values
	gaitdata gait;
	gait_init(&gait, GAIT_TYPE_AMBLE);
	gait_process(&gait);
	
	// initalize controller
	controllerdata controller;
	controller_init();
	controller.x = 0.0;
	controller.y = 0.0;
 	controller.z = 0.0;
	controller.r = 0.0;
	controller.s = 0.0;
	
	// initalize dynamixel bus
	dynamixel_init();
	
	// initalize auto targeting camera
	camera_init();
	
	while(1)
	{	
		gait_process(&gait);
		foot_position_calc(&foot[0], &controller, &gait);
		gait_increment(&gait);
		
		for(uint8_t i = 0; i < 4; i++)
			kinematics_legik(foot[i].x, foot[i].y, foot[i].z, &joint[i*4].angle, &joint[i*4+1].angle, &joint[i*4+2].angle, &joint[i*4+3].angle);
			
		joint[16].angle = 0;
		joint[17].angle = 0;

		joint_write(&joint[0]);
		
		_delay_ms(5);
	}
	
	return 0;
}
