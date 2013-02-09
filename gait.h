
#ifndef _GAIT_H_
#define _GAIT_H_

#include "types.h"

#define GAIT_TYPE_RIPPLE  1
#define GAIT_TYPE_AMBLE   2

typedef struct
{
	u16 period;	               // 
	u16 position;              // 

	u08 step_to_move_ratio;    // dec2
	u08 start_position[4];     // dec2
	
	u16 step_period;           //
	u16 move_period;           // 
	
	u16 step_start[4];         // 
	u16 step_end[4];           //  
	
	s16 tran[4];               // Leg translation result. dec4
	s16 lift[4];               // Leg lift result. dec4
} GAIT;

extern GAIT gait;

void gait_init(GAIT *g, u08 type);
void gait_process(GAIT *g);
void gait_increment(GAIT *g, u16 step_size);
void gait_decrement(GAIT *g, u16 step_size);

#endif