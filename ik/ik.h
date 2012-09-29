
#ifndef _IK_H_
#define _IK_H_

#include <avr/io.h>

uint8_t ik_leg(float x, float y, float z, float* coxa, float* femur, float* tibia, float* tarsus);

#endif
