
#include "kinematics.h"
#include "types.h"
#include "common.h"
#include "okmath.h"

/*
	x - X position of the leg's foot in mm. DEC1
	y - Y position of the leg's foot in mm. DEC1
	z - Z position of the leg's foot in mm. DEC1
	*coxa   - Pointer to where the resulting coxa angle in degrees should be saved. DEC1
	*femur  - Pointer to where the resulting femur angle in degrees should be saved. DEC1
	*tibia  - Pointer to where the resulting tibia angle in degrees should be saved. DEC1
	*tarsus - Pointer to where the resulting tarsus angle in degrees should be saved. DEC1
*/
u08 kinematics_leg_ik(s16 x, s16 y, s16 z, s16 *coxa, s16 *femur, s16 *tibia, s16 *tarsus)
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
	
	// Should do some error checking here!
	
	*coxa = a1;
	*femur = a2;
	*tibia = a3;
	*tarsus = a4;
	
	return 1;
}
