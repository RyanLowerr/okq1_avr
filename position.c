
#include "position.h"
#include "types.h"
#include "common.h"

POSITION current;
POSITION goal;
POSITION coxaoffset;
POSITION neutral;

INTERPOLATION interpolation;

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
	p->turret[0].x = 0;
	p->turret[0].y = 1;
	p->turret[0].z = 0;
}

void position_set_neutral_guns(POSITION *p)
{
	p->gun[0].x = 0;
	p->gun[0].y = 1;
	p->gun[0].z = 0;

	p->gun[1].x = 0;
	p->gun[1].y = 1;
	p->gun[1].z = 0;
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
	position_set_standing(&neutral);
}

static s16 interpolation_calc_delta(s16 x, s16 y)
{

	s16 absx = (x < 0) ? -x : x;
	s16 absy = (y < 0) ? -x : x;
	return ((x + y) < x) ? -(absx + absy) : (absx + absy);
}

u08 interpolation_init(INTERPOLATION *I, POSITION *p1, POSITION *p2, u08 ignore_mask)
{
	u08 n = 0;
	I->ignore_mask = ignore_mask;
	I->period = MAX_U16;;
	I->position = 0;

	position_copy(p1, &(I->ps));
	position_copy(p2, &(I->pe));

	for(u08 i = 0; i < NUM_LEGS; i++)
	{
		if(I->ignore_mask)
		{
			I->delta[n++] = 0;
			I->delta[n++] = 0;
			I->delta[n++] = 0;
		}
		else
		{
			I->delta[n++] = interpolation_calc_delta(p1->foot[i].x, p2->foot[i].x);
			I->delta[n++] = interpolation_calc_delta(p1->foot[i].y, p2->foot[i].y);
			I->delta[n++] = interpolation_calc_delta(p1->foot[i].z, p2->foot[i].z);
		}
	}

	for(u08 i = 0; i < NUM_TURRETS; i++)
	{
		if(I->ignore_mask)
		{
			I->delta[n++] = 0;
			I->delta[n++] = 0;
			I->delta[n++] = 0;
		}
		else
		{
			I->delta[n++] = interpolation_calc_delta(p1->turret[i].x, p2->turret[i].x);
			I->delta[n++] = interpolation_calc_delta(p1->turret[i].y, p2->turret[i].y);
			I->delta[n++] = interpolation_calc_delta(p1->turret[i].z, p2->turret[i].z);
		}
	}

	for(u08 i = 0; i < NUM_GUNS; i++)
	{
		if(I->ignore_mask)
		{
			I->delta[n++] = 0;
			I->delta[n++] = 0;
			I->delta[n++] = 0;
		}
		else
		{
			I->delta[n++] = interpolation_calc_delta(p1->gun[i].x, p2->gun[i].x);
			I->delta[n++] = interpolation_calc_delta(p1->gun[i].y, p2->gun[i].y);
			I->delta[n++] = interpolation_calc_delta(p1->gun[i].z, p2->gun[i].z);
		}
	}

	return 0;
}

u08 interpolation_step(INTERPOLATION *I, POSITION *p, u08 step_size)
{
	u08 n;
	u16 percent;
	
	if((u32)I->position + step_size > I->period)
		I->position = I->period;
	else
		I->position += step_size;
	
	percent = ((u32)I->position * DEC4) / I->period; // DEC4
	n = 0;

	for(u08 i = 0; i < NUM_LEGS; i++)
	{
		if(I->ignore_mask)
		{
			p->foot[n].x += 0; n++;
			p->foot[n].y += 0; n++;
			p->foot[n].z += 0; n++;
		}
		else
		{
			p->foot[n].x = I->ps.foot[n].x + (((s32)I->delta[n] * percent) / DEC4); n++;
			p->foot[n].y = I->ps.foot[n].y + (((s32)I->delta[n] * percent) / DEC4); n++;
			p->foot[n].z = I->ps.foot[n].z + (((s32)I->delta[n] * percent) / DEC4); n++;
		}
	}

	for(u08 i = 0; i < NUM_TURRETS; i++)
	{
		if(I->ignore_mask)
		{
			p->turret[n].x += 0; n++;
			p->turret[n].y += 0; n++;
			p->turret[n].z += 0; n++;	
		}
		else
		{
			p->turret[n].x = I->ps.turret[n].x + (((s32)I->delta[n] * percent) / DEC4); n++;
			p->turret[n].y = I->ps.turret[n].x + (((s32)I->delta[n] * percent) / DEC4); n++;
			p->turret[n].z = I->ps.turret[n].x + (((s32)I->delta[n] * percent) / DEC4); n++;
		}
	}

	for(u08 i = 0; i < NUM_GUNS; i++)
	{
		if(I->ignore_mask)
		{
			p->gun[n].x += 0; n++;
			p->gun[n].y += 0; n++;
			p->gun[n].z += 0; n++;
		}
		else
		{
			p->gun[n].x = I->ps.gun[n].x + (((s32)I->delta[n] * percent) / DEC4); n++;
			p->gun[n].y = I->ps.gun[n].y + (((s32)I->delta[n] * percent) / DEC4); n++;
			p->gun[n].z = I->ps.gun[n].z + (((s32)I->delta[n] * percent) / DEC4); n++;
		}
	}

	if(I->position == I->period)
		return 1;
	else
		return 0;
}
