
#include <math.h>

#include "ik.h"
#include "common.h"

uint8_t ik_leg(float x, float y, float z, ikdata *ik)
{
	float leg_length;
	float tarsus_offset_angle;
	float tarsus_offset_xy;
	float tarsus_offset_z;
	float theta;
	float side_a, side_b, side_c;
	float side_a_sqr, side_b_sqr, side_c_sqr;
	float angle_a, angle_b, angle_c;
	float temp1, temp2;

	// Magnitude of the leg length along the ground from the coxa axis to the tip of the foot in the XY plane
	leg_length = sqrt(x * x + y * y);
	
	// Tarsus Offsets
	tarsus_offset_angle = 0.0; // 0.0 for now. will revisit later.
	tarsus_offset_xy = sin(tarsus_offset_angle) * TARSUS_LENGTH;
	tarsus_offset_z = cos(tarsus_offset_angle) * TARSUS_LENGTH;
	
	// Triangle formed by the femur, tibia and tarsus joints. Using the law of cosines to calculate all sides lengths and angles.
	temp1 = leg_length - COXA_LENGTH - tarsus_offset_xy;
	temp2 = z + tarsus_offset_z;
	
	side_a = FEMUR_LENGTH;
	side_b = TIBIA_LENGTH;
	side_c = sqrt(temp1 * temp1 + temp2 * temp2);
	
	side_a_sqr = side_a * side_a;
	side_b_sqr = side_b * side_b;
	side_c_sqr = side_c * side_c;
	
	//angle_a = acos((-side_a_sqr + side_b_sqr + side_c_sqr) / (2 * side_b * side_c)) * 57.32; // not needed
	angle_b = acos(( side_a_sqr - side_b_sqr + side_c_sqr) / (2 * side_a * side_c)) * 57.32;
	angle_c = acos(( side_a_sqr + side_b_sqr - side_c_sqr) / (2 * side_a * side_b)) * 57.32;
	
	// Angle of line between the femur and Tarsus joints with respect to ground
	theta = atan2(temp2, temp1) * 57.32;
	
	// Resulting joint angles
	ik->coxa   = atan2(x,y) * 57.32;
	ik->femur  = 90.0 - theta - angle_b;
	ik->tibia  = 90.0 - angle_c;
	ik->tarsus = tarsus_offset_angle - ik->femur - ik->tibia;
	
	return 1;
}
