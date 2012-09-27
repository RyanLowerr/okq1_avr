
#ifndef _IK_H_
#define _IK_H_

#include <avr/io.h>

typedef struct
{
	float coxa;
	float femur;
	float tibia;
	float tarsus;
} ikdata;

uint8_t ik_leg(float x, float y, float z, ikdata *ik);
uint8_t ik_body(float roll, float pitch, float yaw);

#endif
