
#ifndef _TYPES_H_
#define _TYPES_H_

#include <avr/io.h>

typedef int8_t   s08;
typedef uint8_t  u08;
typedef int16_t  s16;
typedef uint16_t u16;
typedef int32_t  s32;
typedef uint32_t u32;

#define MAX_U08  255
#define MAX_U16  65535
#define MAX_U32  4294967295

#define MIN_S08 -128
#define MAX_S08  127
#define MIN_S16 -32768
#define MAX_S16  32767
#define MIN_S32 -2147483648
#define MAX_S32  2147483647 

#define DEC1 10
#define DEC2 100
#define DEC3 1000
#define DEC4 10000
#define DEC5 100000
#define DEC6 1000000

#endif
