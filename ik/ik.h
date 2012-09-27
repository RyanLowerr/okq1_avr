
#ifndef _IK_H_
#define _IK_H_

#include <avr/io.h>

typedef struct
{
	float coxa;
	float femur;
	float tibia;
	float tarsus;
} ik_angles;

uint8_t ik_leg(float x, float y, float z, ik_angles *results);
uint8_t ik_body(float roll, float pitch, float yaw);

#endif
