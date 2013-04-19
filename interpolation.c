
#include "interpolation.h"
#include "position.h"
#include "types.h"
#include "common.h"

INTERPOLATION interp_legs;
INTERPOLATION interp_turret;

static s16 interpolation_calc_delta(s16 x, s16 y)
{
	s16 absx = (x < 0) ? -x : x;
	s16 absy = (y < 0) ? -y : y;
	s16 delta = (absx > absy) ? (absx - absy) : (absy - absx);
	return (x > y) ? -delta : delta;
}

u08 interpolation_init(INTERPOLATION *I, POSITION *p1, POSITION *p2, u08 ignore_mask)
{
	u08 n = 0;
	I->ignore_mask = ignore_mask;
	I->period = MAX_U16;;
	I->position = 0;
	
	I->ps = *p1;
	I->pe = *p2;
	
	for(u08 i = 0; i < NUM_LEGS; i++)
	{
		if(I->ignore_mask & INTERPOLATION_IGNORE_LEGS)
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
	
	if(I->ignore_mask & INTERPOLATION_IGNORE_TURRET)
	{
		I->delta[n++] = 0;
		I->delta[n++] = 0;
		I->delta[n++] = 0;
	}
	else
	{
		I->delta[n++] = interpolation_calc_delta(p1->turret.x, p2->turret.x);
		I->delta[n++] = interpolation_calc_delta(p1->turret.y, p2->turret.y);
		I->delta[n++] = interpolation_calc_delta(p1->turret.z, p2->turret.z);
	}
	
	return 0;
}

u08 interpolation_step(INTERPOLATION *I, POSITION *p, u16 step_size)
{
	u08 n;
	u16 percent;
	
	if((u32)I->position + step_size >= I->period)
		I->position = I->period;
	else
		I->position += step_size;
	
	percent = ((u32)I->position * DEC4) / I->period; // DEC4
	n = 0;
	
	for(u08 i = 0; i < NUM_LEGS; i++)
	{
		if(I->ignore_mask & INTERPOLATION_IGNORE_LEGS)
		{
			p->foot[i].x += 0; n++;
			p->foot[i].y += 0; n++;
			p->foot[i].z += 0; n++;
		}
		else
		{
			p->foot[i].x = I->ps.foot[i].x + (((s32)I->delta[n] * percent) / DEC4); n++;
			p->foot[i].y = I->ps.foot[i].y + (((s32)I->delta[n] * percent) / DEC4); n++;			
			p->foot[i].z = I->ps.foot[i].z + (((s32)I->delta[n] * percent) / DEC4); n++;
		}
	}
	
	if(I->ignore_mask & INTERPOLATION_IGNORE_TURRET)
	{
		p->turret.x += 0; n++;
		p->turret.y += 0; n++;
		p->turret.z += 0; n++;	
	}
	else
	{
		p->turret.x = I->ps.turret.x + (((s32)I->delta[n] * percent) / DEC4); n++;
		p->turret.y = I->ps.turret.y + (((s32)I->delta[n] * percent) / DEC4); n++;
		p->turret.z = I->ps.turret.z + (((s32)I->delta[n] * percent) / DEC4); n++;
	}
	
	return (I->position == I->period) ? 1 : 0;
}