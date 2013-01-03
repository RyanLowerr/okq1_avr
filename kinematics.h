
#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include "types.h"

u08 kinematics_legik(float x, float y, float z, float* coxa, float* femur, float* tibia, float* tarsus);
u08 kinematics_legfk(float a1, float a2, float a3, float a4);
u08 kinematics_turretik(void);
u08 kinematics_gunik(void);

#endif
