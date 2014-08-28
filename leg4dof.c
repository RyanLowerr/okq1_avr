
#include "leg4dof.h"
#include "types.h"
#include "common.h"
#include "okmath.h"
#include "position.h"

INTERPOLATION interp;

/*
	x - X position of the leg's foot in mm. DEC1
	y - Y position of the leg's foot in mm. DEC1
	z - Z position of the leg's foot in mm. DEC1
	*coxa   - Pointer to where the resulting coxa angle in degrees should be saved. DEC1
	*femur  - Pointer to where the resulting femur angle in degrees should be saved. DEC1
	*tibia  - Pointer to where the resulting tibia angle in degrees should be saved. DEC1
	*tarsus - Pointer to where the resulting tarsus angle in degrees should be saved. DEC1
*/
u08 leg4dof_kinematics_reverse(s16 x, s16 y, s16 z, s16 *coxa, s16 *femur, s16 *tibia, s16 *tarsus)
{ 
	// Magnitude of a vector along the ground from the coxa axis to the tip of the foot. DEC1
	u32 leg_length = okmath_sqrt(((s32)x * x) + ((s32)y * y));

	// We are using the law of cosines on the triangle formed by the femur, tibia and tarsus joints.
	s32 femur_to_tarsus = leg_length - COXA_LENGTH;
	s32 tarsus_to_zero_z = z + TARSUS_LENGTH;
	
	// Length of the sides of the triangle formed by the femur, tibia and tarsus joints. DEC1
	s32 side_a = FEMUR_LENGTH;
	s32 side_a_sqr = (side_a * side_a) / DEC1;
	
	s32 side_b = TIBIA_LENGTH;
	s32 side_b_sqr = (side_b * side_b) / DEC1;
	
	s32 side_c = okmath_sqrt(femur_to_tarsus * femur_to_tarsus + tarsus_to_zero_z * tarsus_to_zero_z);
	s32 side_c_sqr = (side_c * side_c) / DEC1;
	
	// The angles of the triangle formed by the femur, tibia and tarsus joints. DEC1
	s16 angle_b = ((s32)okmath_acos(((side_a_sqr - side_b_sqr + side_c_sqr) * DEC4) / ((2 * side_a * side_c) / DEC1)) * 180) / 3141;
	s16 angle_c = ((s32)okmath_acos(((side_a_sqr + side_b_sqr - side_c_sqr) * DEC4) / ((2 * side_a * side_b) / DEC1)) * 180) / 3141;
	
	// Angle of line between the femur and Tarsus joints with respect to ground. DEC1
	s16 theta = ((s32)okmath_acos((femur_to_tarsus * DEC4) / side_c) * 180) / 3141;
	
	// Resulting joint angles in degrees. DEC1
	s16 a1 = ((s32)okmath_acos(((s32)x * DEC4) / (s32)leg_length) * 180) / 3141; // coxa
	s16 a2 = 900 - theta - angle_b;                                              // femur
	s16 a3 = 900 - angle_c;                                                      // tibia
	s16 a4 = 0 - a2 - a3;                                                        // tarsus
	
	*coxa = (x < 0) ? (1800 - a1) : a1;	
	*femur = a2;
	*tibia = a3;
	*tarsus = a4;
	
	return 1;
}

u08 leg4dof_kinematics_forward(void)
{
	return 0;
}

static s16 interpolation_calc_delta(s16 x, s16 y)
{
	s16 absx = (x < 0) ? -x : x;
	s16 absy = (y < 0) ? -y : y;
	s16 delta = (absx > absy) ? (absx - absy) : (absy - absx);
	return (x > y) ? -delta : delta;
}

u08 leg4dof_interpolation_init(INTERPOLATION *I, POSITION *p1, POSITION *p2)
{
	u08 n = 0;
	I->period = MAX_U16;;
	I->position = 0;
	
	I->ps = *p1;
	I->pe = *p2;
	
	for(u08 i = 0; i < NUM_LEGS; i++)
	{
		I->delta[n++] = interpolation_calc_delta(p1->foot[i].x, p2->foot[i].x);
		I->delta[n++] = interpolation_calc_delta(p1->foot[i].y, p2->foot[i].y);
		I->delta[n++] = interpolation_calc_delta(p1->foot[i].z, p2->foot[i].z);
	}
	
	return 0;
}

u08 leg4dof_interpolation_step(INTERPOLATION *I, POSITION *p, u16 step_size)
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
		p->foot[i].x = I->ps.foot[i].x + (((s32)I->delta[n] * percent) / DEC4); n++;
		p->foot[i].y = I->ps.foot[i].y + (((s32)I->delta[n] * percent) / DEC4); n++;			
		p->foot[i].z = I->ps.foot[i].z + (((s32)I->delta[n] * percent) / DEC4); n++;
	}
	
	return (I->position == I->period) ? 1 : 0;
}
