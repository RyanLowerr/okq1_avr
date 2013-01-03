
#ifndef _OKIRTRAK_H_
#define _OKIRTRAK_H_

#include "types.h"

typedef struct
{
	u08 address;

	u16 x[4];
	u08 xmin[4];
	u08 xmax[4];
	
	u16 y[4];
	u08 ymin[4];
	u08 ymax[4];
	
	u08 s[4];
	u08 I[4];
} OKIRTRAK;

void okirtrak_init(OKIRTRAK *cam, u08 address);
void okirtrak_process(OKIRTRAK *cam);

#endif
