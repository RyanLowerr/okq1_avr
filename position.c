
#include "position.h"
#include "types.h"
#include "common.h"

POSITION current;
POSITION goal;
POSITION coxaoffset;
POSITION neutral;

void position_copy(POSITION *pi, POSITION *po)
{
	for(u08 i = 0; i < NUM_LEGS; i++)
	{
		po->foot[i].x = pi->foot[i].x;
		po->foot[i].y = pi->foot[i].y;
		po->foot[i].z = pi->foot[i].z;
	}

	for(u08 i = 0; i < NUM_TURRETS; i++)
	{
		po->turret[i].x = pi->turret[i].x;
		po->turret[i].y = pi->turret[i].y;
		po->turret[i].z = pi->turret[i].z;
	}

	for(u08 i = 0; i < NUM_GUNS; i++)
	{
		po->gun[i].x = pi->gun[i].x;
		po->gun[i].y = pi->gun[i].y;
		po->gun[i].z = pi->gun[i].z;
	}	
}

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
}