
#ifndef _IK_H_
#define _IK_H_

#include <avr/io.h>

#define COXA_LENGTH   46.0
#define FEMUR_LENGTH  70.0
#define TIBIA_LENGTH  68.0
#define TARSUS_LENGTH 83.0

typedef struct
{
	double coxa;
	double femur;
	double tibia;
	double tarsus;
} ik_angles;

uint8_t ik_leg(double x, double y, double z, ik_angles *results);

#endif
