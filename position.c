
#include "position.h"
#include "types.h"
#include "common.h"
#include "okmath.h"

POSITION current;
POSITION goal;
POSITION coxaoffset;
POSITION neutral;

void position_copy(POSITION *pi, POSITION *po)
{
	po->legignore = pi->legignore;
	for(u08 i = 0; i < NUM_LEGS; i++)
	{
		po->foot[i].x = pi->foot[i].x;
		po->foot[i].y = pi->foot[i].y;
		po->foot[i].z = pi->foot[i].z;
	}

	po->turretignore = pi->turretignore;
	for(u08 i = 0; i < NUM_TURRETS; i++)
	{
		po->turret[i].x = pi->turret[i].x;
		po->turret[i].y = pi->turret[i].y;
		po->turret[i].z = pi->turret[i].z;
	}

	po->gunignore = pi->gunignore;
	for(u08 i = 0; i < NUM_GUNS; i++)
	{
		po->gun[i].x = pi->gun[i].x;
		po->gun[i].y = pi->gun[i].y;
		po->gun[i].z = pi->gun[i].z;
	}	
}

void position_set_goal(POSITION *p)
{
	position_copy(p, &goal);
}

void position_set_standing(POSITION *p)
{
	p->foot[0].x =  FOOT_X_NEUTRAL;
	p->foot[0].y =  FOOT_Y_NEUTRAL;
	p->foot[0].z = -FOOT_Z_NEUTRAL;

	p->foot[1].x =  FOOT_X_NEUTRAL;
	p->foot[1].y = -FOOT_Y_NEUTRAL;
	p->foot[1].z = -FOOT_Z_NEUTRAL;

	p->foot[2].x = -FOOT_X_NEUTRAL;
	p->foot[2].y = -FOOT_Y_NEUTRAL;
	p->foot[2].z = -FOOT_Z_NEUTRAL;

	p->foot[3].x = -FOOT_X_NEUTRAL;
	p->foot[3].y =  FOOT_Y_NEUTRAL;
	p->foot[3].z = -FOOT_Z_NEUTRAL;
}

void position_set_sitting(POSITION *p)
{
	;
}

void position_set_neutral_turrets(POSITION *p)
{
	p->turret[0].x = 0.0;
	p->turret[0].y = 1.0;
	p->turret[0].z = 0.0;
}

void position_set_neutral_guns(POSITION *p)
{
	p->gun[0].x = 0.0;
	p->gun[0].y = 1.0;
	p->gun[0].z = 0.0;

	p->gun[1].x = 0.0;
	p->gun[1].y = 1.0;
	p->gun[1].z = 0.0;
}

void position_init(void)
{
	// coxa offset position initilization.
	coxaoffset.foot[0].x =  COXA_X_OFFSET;
	coxaoffset.foot[0].y =  COXA_X_OFFSET;
	coxaoffset.foot[0].z =  0.0;

	coxaoffset.foot[1].x =  COXA_X_OFFSET;
	coxaoffset.foot[1].y = -COXA_X_OFFSET;
	coxaoffset.foot[1].z =  0.0;

	coxaoffset.foot[2].x = -COXA_X_OFFSET;
	coxaoffset.foot[2].y = -COXA_X_OFFSET;
	coxaoffset.foot[2].z =  0.0;

	coxaoffset.foot[3].x = -COXA_X_OFFSET;
	coxaoffset.foot[3].y =  COXA_X_OFFSET;
	coxaoffset.foot[3].z =  0.0;

	coxaoffset.legignore = 0;
	coxaoffset.turretignore = 255;
	coxaoffset.gunignore = 255;

	// neutral leg position initilization
	position_set_standing(&neutral);
	
	neutral.legignore = 0;
	neutral.turretignore = 255;
	neutral.gunignore = 255;
	
	// current position initilization
	position_set_standing(&current);
	position_set_neutral_turrets(&current);
	position_set_neutral_guns(&current);
	
	current.legignore = 0;
	current.turretignore = 0;
	current.gunignore = 0;
}

u08 interpolation_init(INTERPOLATION *I, POSITION *p1, POSITION *p2, u16 period)
{
	u08 n = 0;

	I->period = period;
	I->position = 0;

	position_copy(p1, &(I->ps));
	position_copy(p2, &(I->pe));

	for(u08 i = 0; i < NUM_LEGS; i++)
	{
		if(I->pe.legignore & (1 << i))
		{
			I->stepsize[n++] = (p1->foot[i].x - p2->foot[i].x) / (float)I->period;
			I->stepsize[n++] = (p1->foot[i].y - p2->foot[i].y) / (float)I->period;
			I->stepsize[n++] = (p1->foot[i].z - p2->foot[i].z) / (float)I->period;
		}
		else
		{
			I->stepsize[n++] = 0.0;
			I->stepsize[n++] = 0.0;
			I->stepsize[n++] = 0.0;
		}
	}

	for(u08 i = 0; i < NUM_TURRETS; i++)
	{
		if(I->pe.turretignore & (1 << i))
		{
			I->stepsize[n++] = (p1->turret[i].x - p2->turret[i].x) / (float)I->period;
			I->stepsize[n++] = (p1->turret[i].y - p2->turret[i].y) / (float)I->period;
			I->stepsize[n++] = (p1->turret[i].z - p2->turret[i].z) / (float)I->period;
		}
		else
		{
			I->stepsize[n++] = 0.0;
			I->stepsize[n++] = 0.0;
			I->stepsize[n++] = 0.0;
		}
	}

	for(u08 i = 0; i < NUM_GUNS; i++)
	{
		if(I->pe.gunignore & (1 << i))
		{
			I->stepsize[n++] = (p1->gun[i].x - p2->gun[i].x) / (float)I->period;
			I->stepsize[n++] = (p1->gun[i].y - p2->gun[i].y) / (float)I->period;
			I->stepsize[n++] = (p1->gun[i].z - p2->gun[i].z) / (float)I->period;
		}
		else
		{
			I->stepsize[n++] = 0.0;
			I->stepsize[n++] = 0.0;
			I->stepsize[n++] = 0.0;
		}
	}

	return 0;
}

u08 interpolation_step(INTERPOLATION *I, POSITION *p)
{
	u08 n = 0;

	for(u08 i = 0; i < NUM_LEGS; i++)
	{
		p->foot[n].x += I->stepsize[n]; n++;
		p->foot[n].y += I->stepsize[n]; n++;
		p->foot[n].z += I->stepsize[n]; n++;
	}

	for(u08 i = 0; i < NUM_TURRETS; i++)
	{
		p->turret[n].x += I->stepsize[n]; n++;
		p->turret[n].y += I->stepsize[n]; n++;
		p->turret[n].z += I->stepsize[n]; n++;
	}

	for(u08 i = 0; i < NUM_GUNS; i++)
	{
		p->gun[n].x += I->stepsize[n]; n++;
		p->gun[n].y += I->stepsize[n]; n++;
		p->gun[n].z += I->stepsize[n]; n++;
	}

	if(I->position == I->period)
	{
		return 1;
	}
	else
	{
		I->position += 1;
		return 0;
	}
}