
#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include "types.h"

u08 kinematics_leg_ik(s16 x, s16 y, s16 z, s16 *coxa, s16 *femur, s16 *tibia, s16 *tarsus);
u08 kinematics_leg_fk(void);
u08 kinematics_turret_ik(void);
u08 kinematics_turret_fk(void);

#endif