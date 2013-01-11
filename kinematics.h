
#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include "types.h"

u08 kinematics_leg_ik(s32 x, s32 y, s32 z, s16 *coxa, s16 *femur, s16 *tibia, s16 *tarsus);
u08 kinematics_leg_fk(void);
u08 kinematics_turret_ik(void);
u08 kinematics_turret_fk(void);
u08 kinematics_gun_ik(void);
u08 kinematics_gun_fk(void);

#endif
