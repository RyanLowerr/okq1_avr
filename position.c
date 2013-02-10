
#include "position.h"
#include "types.h"
#include "common.h"

POSITION current;
POSITION goal;
POSITION coxaoffset;
POSITION neutral;

void position_init(void)
{
	// coxa offset position initilization.
	coxaoffset.foot[0].x =  COXA_X_OFFSET;
	coxaoffset.foot[0].y =  COXA_X_OFFSET;
	coxaoffset.foot[0].z =  0;
	
	coxaoffset.foot[1].x =  COXA_X_OFFSET;
	coxaoffset.foot[1].y = -COXA_X_OFFSET;
	coxaoffset.foot[1].z =  0;
	
	coxaoffset.foot[2].x = -COXA_X_OFFSET;
	coxaoffset.foot[2].y = -COXA_X_OFFSET;
	coxaoffset.foot[2].z =  0;
	
	coxaoffset.foot[3].x = -COXA_X_OFFSET;
	coxaoffset.foot[3].y =  COXA_X_OFFSET;
	coxaoffset.foot[3].z =  0;
	
	// neutral leg position initilization
	neutral.foot[0].x =  FOOT_X_NEUTRAL;
	neutral.foot[0].y =  FOOT_Y_NEUTRAL;
	neutral.foot[0].z = -FOOT_Z_NEUTRAL;
	
	neutral.foot[1].x =  FOOT_X_NEUTRAL;
	neutral.foot[1].y = -FOOT_Y_NEUTRAL;
	neutral.foot[1].z = -FOOT_Z_NEUTRAL;
	
	neutral.foot[2].x = -FOOT_X_NEUTRAL;
	neutral.foot[2].y = -FOOT_Y_NEUTRAL;
	neutral.foot[2].z = -FOOT_Z_NEUTRAL;
	
	neutral.foot[3].x = -FOOT_X_NEUTRAL;
	neutral.foot[3].y =  FOOT_Y_NEUTRAL;
	neutral.foot[3].z = -FOOT_Z_NEUTRAL;
	
	// current leg position initilization
	current.foot[0].x =  FOOT_X_NEUTRAL;
	current.foot[0].y =  FOOT_Y_NEUTRAL;
	current.foot[0].z = -FOOT_Z_NEUTRAL;
	
	current.foot[1].x =  FOOT_X_NEUTRAL;
	current.foot[1].y = -FOOT_Y_NEUTRAL;
	current.foot[1].z = -FOOT_Z_NEUTRAL;
	
	current.foot[2].x = -FOOT_X_NEUTRAL;
	current.foot[2].y = -FOOT_Y_NEUTRAL;
	current.foot[2].z = -FOOT_Z_NEUTRAL;
	
	current.foot[3].x = -FOOT_X_NEUTRAL;
	current.foot[3].y =  FOOT_Y_NEUTRAL;
	current.foot[3].z = -FOOT_Z_NEUTRAL;
}