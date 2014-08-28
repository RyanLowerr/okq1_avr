
#ifndef _POSITION_H_
#define _POSITION_H_

#include "types.h"
#include "common.h"
#include "okmath.h"

typedef struct {
	VECTOR foot[NUM_LEGS];
} POSITION;

extern POSITION current;
extern POSITION goal;
extern POSITION coxaoffset;
extern POSITION neutral;

void position_init(void);

#endif
