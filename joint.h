
#ifndef _JOINT_H_
#define _JOINT_H_

#include "common.h"

typedef struct
{
	float angle;
	uint16_t position;
	uint16_t prevposition;
	uint16_t center;
	uint8_t id;
	int8_t direction;
	int8_t type;
} JOINT;

#define JOINT_TYPE_AX 1
#define JOINT_TYPE_MX 2

extern JOINT joint[NUM_SERVOS];

void joint_init(JOINT *joint);
void joint_write(JOINT *joint);

#endif
