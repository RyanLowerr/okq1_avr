
#include "interpolation.h"
#include "position.h"
#include "types.h"
#include "common.h"

INTERPOLATION interpolation;

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

u08 interpolation_step(INTERPOLATION *I, POSITION *p, u16 step_size)
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
	
	for(u08 i = 0; i < NUM_TURRETS; i++)
	{
		if(I->ignore_mask)
		{
			p->turret[i].x += 0; n++;
			p->turret[i].y += 0; n++;
			p->turret[i].z += 0; n++;	
		}
		else
		{
			p->turret[i].x = I->ps.turret[i].x + (((s32)I->delta[n] * percent) / DEC4); n++;
			p->turret[i].y = I->ps.turret[i].y + (((s32)I->delta[n] * percent) / DEC4); n++;
			p->turret[i].z = I->ps.turret[i].z + (((s32)I->delta[n] * percent) / DEC4); n++;
		}
	}
	
	for(u08 i = 0; i < NUM_GUNS; i++)
	{
		if(I->ignore_mask)
		{
			p->gun[i].x += 0; n++;
			p->gun[i].y += 0; n++;
			p->gun[i].z += 0; n++;
		}
		else
		{
			p->gun[i].x = I->ps.gun[i].x + (((s32)I->delta[n] * percent) / DEC4); n++;
			p->gun[i].y = I->ps.gun[i].y + (((s32)I->delta[n] * percent) / DEC4); n++;
			p->gun[i].z = I->ps.gun[i].z + (((s32)I->delta[n] * percent) / DEC4); n++;
		}
	}
	
	if(I->position != I->period)
		return 1;
	else
		return 0;
}
