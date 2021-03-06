
#ifndef _COMMON_H_
#define _COMMON_H_

#define NUM_SERVOS 22
#define NUM_LEGS 4

// Leg segment lengths (mm from joint to joint). DEC1
#define COXA_LENGTH   600
#define FEMUR_LENGTH  900
#define TIBIA_LENGTH  800
#define TARSUS_LENGTH 800

// Foot neutral positions (mm from center of coxa joint). DEC1
#define FOOT_X_NEUTRAL 1300
#define FOOT_Y_NEUTRAL 800
#define FOOT_Z_NEUTRAL 600

// Foot siting position (mm from center of coxa joint). DEC1
#define FOOT_Z_SITTING 200

// Coxa offset from center (mm from center of robot). DEC1
#define COXA_X_OFFSET 600
#define COXA_Y_OFFSET 600

// Servo ID's
#define FR_COXA_ID     1
#define FR_FEMUR_ID    2
#define FR_TIBIA_ID    3
#define FR_TARSUS_ID   4

#define BR_COXA_ID     5
#define BR_FEMUR_ID    6
#define BR_TIBIA_ID    7
#define BR_TARSUS_ID   8

#define BL_COXA_ID     9
#define BL_FEMUR_ID    10
#define BL_TIBIA_ID    11
#define BL_TARSUS_ID   12

#define FL_COXA_ID     13
#define FL_FEMUR_ID    14
#define FL_TIBIA_ID    15
#define FL_TARSUS_ID   16

// Servo Direction
#define FR_COXA_DIRECTION     1
#define FR_FEMUR_DIRECTION    1
#define FR_TIBIA_DIRECTION   -1
#define FR_TARSUS_DIRECTION  -1

#define BR_COXA_DIRECTION    -1
#define BR_FEMUR_DIRECTION    1
#define BR_TIBIA_DIRECTION   -1
#define BR_TARSUS_DIRECTION  -1

#define BL_COXA_DIRECTION     1
#define BL_FEMUR_DIRECTION   -1
#define BL_TIBIA_DIRECTION    1
#define BL_TARSUS_DIRECTION   1

#define FL_COXA_DIRECTION    -1
#define FL_FEMUR_DIRECTION   -1
#define FL_TIBIA_DIRECTION    1
#define FL_TARSUS_DIRECTION   1

#define TURRET_PAN_DIRECTION  1
#define TURRET_TILT_DIRECTION 1

#define R_GUN_PAN_DIRECTION   1
#define R_GUN_TILT_DIRECTION  1
#define L_GUN_PAN_DIRECTION   1
#define L_GUN_TILT_DIRECTION  1

// Servo 'Zero' angle dec1
#define FR_COXA_ZERO     -900
#define BR_COXA_ZERO      900
#define BL_COXA_ZERO     -900
#define FL_COXA_ZERO      900
#define FEMUR_ZERO        0
#define TIBIA_ZERO        0
#define TARSUS_ZERO       0

#define TURRET_PAN_ZERO   0
#define TURRET_TILT_ZERO  0

#define R_GUN_PAN_ZERO    0
#define R_GUN_TILT_ZERO   0
#define L_GUN_PAN_ZERO    0
#define L_GUN_TILT_ZERO   0

#endif
