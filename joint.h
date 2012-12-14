
#ifndef _JOINT_H_
#define _JOINT_H_

typedef struct
{
	uint8_t id;
	uint16_t position;
	int16_t center;
	int direction;
	float angle;
} JOINT;

void joint_init(JOINT* joint);
void joint_write(JOINT* joint);

#endif
