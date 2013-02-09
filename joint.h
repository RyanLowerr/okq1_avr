
#ifndef _JOINT_H_
#define _JOINT_H_

#include "common.h"
#include "types.h"

typedef struct
{
	s16 angle; // DEC1
	u16 position;
	u16 prevposition;
	s16 center;
	u08 id;
	s08 direction;
	s08 type;
} JOINT;

#define JOINT_TYPE_AX 1
#define JOINT_TYPE_MX 2

extern JOINT joint[NUM_SERVOS];

void joint_init(JOINT *joint);
void joint_process(JOINT *joint);

#endif