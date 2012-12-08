
#include <avr/io.h>

#include "foot.h"
#include "common.h"

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
