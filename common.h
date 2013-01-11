
#ifndef _COMMON_H_
#define _COMMON_H_

#define NUM_SERVOS 22
#define NUM_LEGS 4
#define NUM_TURRETS 1
#define NUM_GUNS 2

// Leg segment lengths (mm from joint to joint) dec2
#define COXA_LENGTH   6000
#define FEMUR_LENGTH  9000
#define TIBIA_LENGTH  8000
#define TARSUS_LENGTH 8000

// Foot neutral positions (mm from center of coxa joint) dec2
#define FOOT_X_NEUTRAL 13000
#define FOOT_Y_NEUTRAL 8000
#define FOOT_Z_NEUTRAL 6000

// Coxa offset from center (mm from center of robot) dec2
#define COXA_X_OFFSET 6000
#define COXA_Y_OFFSET 6000

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

#define TURRET_PAN_ID  17
#define TURRET_TILT_ID 18

#define R_GUN_PAN_ID   19
#define R_GUN_TILT_ID  20
#define L_GUN_PAN_ID   21
#define L_GUN_TILT_ID  22

// Servo Direction
#define FR_COXA_DIRECTION     1
#define FR_FEMUR_DIRECTION    1
#define FR_TIBIA_DIRECTION   -1
#define FR_TARSUS_DIRECTION  -1

#define BR_COXA_DIRECTION     1
#define BR_FEMUR_DIRECTION    1
#define BR_TIBIA_DIRECTION   -1
#define BR_TARSUS_DIRECTION  -1

#define BL_COXA_DIRECTION     1
#define BL_FEMUR_DIRECTION   -1
#define BL_TIBIA_DIRECTION    1
#define BL_TARSUS_DIRECTION   1

#define FL_COXA_DIRECTION     1
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
#define FR_COXA_ZERO     0
#define BR_COXA_ZERO     900
#define BL_COXA_ZERO     900
#define FL_COXA_ZERO     00
#define FEMUR_ZERO       00
#define TIBIA_ZERO       00
#define TARSUS_ZERO      00

#define TURRET_PAN_ZERO  0
#define TURRET_TILT_ZERO 0

#define R_GUN_PAN_ZERO   0
#define R_GUN_TILT_ZERO  0
#define L_GUN_PAN_ZERO   0
#define L_GUN_TILT_ZERO  0

#endif
