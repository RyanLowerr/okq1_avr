
#include "kinematics.h"
#include "types.h"
#include "common.h"
#include "okmath.h"

u08 kinematics_leg_ik(s32 x, s32 y, s32 z, s16 *coxa, s16 *femur, s16 *tibia, s16 *tarsus)
{
	u16 leg_length;                         // Magnitude of a vector along the ground from the coxa axis to the tip of the foot. DEC2
	s16 tarsus_offset_angle;                // Angle between the tarsus leg segment and the ground. DEC1
	s16 tarsus_offset_xy;                   // DEC2
	s16 tarsus_offset_z;                    // DEC2
	s16 theta;                              // Angle of line between the femur and Tarsus joints with respect to ground. DEC2 
	u16 side_a, side_b, side_c;             // Sides of the triangle formed by the femur, tibia and tarsus joints. DEC2 
	u16 side_a_sqr, side_b_sqr, side_c_sqr; // Sides of the triangle formed by the femur, tibia and tarsus joints raised to the power of two. DEC2
	s16 angle_b, angle_c;                   // Angles of the triangle formed by the femur, tibia and tarsus joints. DEC1
	s16 result[4];                          // Temp stoarage of the resulting IK angles. DEC1

	// Calculate the leg_length.
	leg_length = okmath_sqrt((x * x + y * y) / DEC4);
	
	// Calculate the tarsus leg segment offsets.
	tarsus_offset_angle = 0; // 0 for now to make the tibia always perpendicular to the ground. Will revisit later.
	tarsus_offset_xy = 0; //okmath_sin(tarsus_offset_angle) * TARSUS_LENGTH / DEC6;
	tarsus_offset_z  = TARSUS_LENGTH; //okmath_cos(tarsus_offset_angle) * TARSUS_LENGTH / DEC6;
	
	// Triangle formed by the femur, tibia and tarsus joints. Using the law of cosines to calculate all sides lengths and angles.
	u16 temp1 = leg_length - COXA_LENGTH - tarsus_offset_xy;
	u16 temp2 = z + tarsus_offset_z;
	
	side_a = FEMUR_LENGTH;
	side_b = TIBIA_LENGTH;
	side_c = okmath_sqrt((temp1 * temp1 + temp2 * temp2) / DEC2);
	
	side_a_sqr = (side_a * side_a) / DEC2;
	side_b_sqr = (side_b * side_b) / DEC2;
	side_c_sqr = (side_c * side_c) / DEC2;
	
	//angle_a not needed
	angle_b = okmath_acos((side_a_sqr - side_b_sqr + side_c_sqr) / (2 * (side_a * side_c) / DEC2));
	angle_c = okmath_acos((side_a_sqr + side_b_sqr - side_c_sqr) / (2 * (side_a * side_b) / DEC2));
	
	// Angle of line between the femur and Tarsus joints with respect to ground
	theta = (okmath_atan2(temp2, temp1) * 5732) / DEC2;
	
	// Resulting joint angles
	result[0] = (okmath_atan2(y,x) * 5732) / DEC2;           // coxa
	result[1] = 900 - theta - angle_b;                       // femur
	result[2] = 900 - angle_c;                               // tibia
	result[3] = tarsus_offset_angle - result[1] - result[2]; // tarsus
	
	// Should do some error checking here!
	
	*coxa = result[0];
	*femur = result[1];
	*tibia = result[2];
	*tarsus = result[3];
	
	return 1;
}

u08 kinematics_leg_fk(void)
{
	return 0;
}

u08 kinematics_turret_ik(void)
{
	return 0;
}

u08 kinematics_turret_fk(void)
{
	return 0;
}

u08 kinematics_gun_ik(void)
{
	return 0;
}

u08 kinematics_gun_fk(void)
{
	return 0;
}
