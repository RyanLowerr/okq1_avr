
#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include <avr/io.h>

uint8_t kinematics_legik(float x, float y, float z, float* coxa, float* femur, float* tibia, float* tarsus);
uint8_t kinematics_legfk(float a1, float a2, float a3, float a4);

#endif
