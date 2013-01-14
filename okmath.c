
#include "okmath.h"
#include "types.h"

u16 okmath_vector_magnitude(VECTOR *v)
{
	return 0;
}

void okmath_vector_normalize(VECTOR *vi, VECTOR *vo)
{
	VECTOR temp;
}

void okmath_vector_rotate(VECTOR *vi, VECTOR *vo, s16 roll, s16 pitch, s16 yaw)
{
	VECTOR temp;
}

/*
	okmath_sqrt()
*/
u32 okmath_sqrt(u32 number)
{
	u32 root = 0;
	u32 num = number;
	u32 bit = 1UL << 30;
	
	// Bit starts at the highest power of four <= to input number.
	while(bit > num)
		bit >>= 2;
		
	while(bit != 0)
	{
		if(num >= root + bit)
		{
			num -= (root + bit);
			root += (bit << 1);
		}
		root >>= 1;
		bit >>= 2;
	}
	
	return root;
}

/*
	Sin lookup table. 0 to 90 degrees with 0.5 degree steps. stored as u16 dec4
*/
static const u16 sintable[] = 
{
	    0,   87,  174,  261,  348,  436,  523,  610,  697,  784,
	  871,  958, 1045, 1132, 1218, 1305, 1391, 1478, 1564, 1650,
	 1736, 1822, 1908, 1993, 2079, 2164, 2249, 2334, 2419, 2503,
	 2588, 2672, 2756, 2840, 2923, 3007, 3090, 3173, 3255, 3338,
	 3420, 3502, 3583, 3665, 3746, 3826, 3907, 3987, 4067, 4146,
	 4226, 4305, 4383, 4461, 4539, 4617, 4694, 4771, 4848, 4924,
	 4999, 5075, 5150, 5224, 5299, 5372, 5446, 5519, 5591, 5664,
	 5735, 5807, 5877, 5948, 6018, 6087, 6156, 6225, 6293, 6360,
	 6427, 6494, 6560, 6626, 6691, 6755, 6819, 6883, 6946, 7009,
	 7071, 7132, 7193, 7253, 7313, 7372, 7431, 7489, 7547, 7604,
	 7660, 7716, 7771, 7826, 7880, 7933, 7986, 8038, 8090, 8141,
	 8191, 8241, 8290, 8338, 8386, 8433, 8480, 8526, 8571, 8616,
	 8660, 8703, 8746, 8788, 8829, 8870, 8910, 8949, 8987, 9025,
	 9063, 9099, 9135, 9170, 9205, 9238, 9271, 9304, 9335, 9366,
	 9396, 9426, 9455, 9483, 9510, 9537, 9563, 9588, 9612, 9636,
	 9659, 9681, 9702, 9723, 9743, 9762, 9781, 9799, 9816, 9832,
	 9848, 9862, 9876, 9890, 9902, 9914, 9925, 9935, 9945, 9953,
	 9961, 9969, 9975, 9981, 9986, 9990, 9993, 9996, 9998, 9999,
	 10000
};

/*
	The function okmath_scaledeg() scales input angle to a value between 0 and 360 degrees. 
	Input deg is an angle in degrees represented as a s16 dec1. (-450 = -45.0 degrees)
	Returns scaled input deg as a u16 dec1. (-450 -> 3150 = 315.0 degrees)
*/
static u16 okmath_scaledeg(s16 deg)
{
	u16 absdeg;
	u16 posdeg;
	 
	if(deg < 0)
	{
		absdeg = -1 * deg;
		posdeg = 3600 - (absdeg - (3600 * (absdeg / 3600)));
	}
	else
	{
		absdeg = deg;
		posdeg = absdeg - (3600 * (absdeg / 3600));
	}
	
	return posdeg;
}


/*
	The functions okmath_sin() and okmath_cos() use sintable to determine the sin or cos value of the input angle.
	Both functions assume input deg is an angle in degrees represented as a s16 dec1. (-450 = -45.0 deg)
	Both functions return a value between -1 and 1 as a s16 dec4. (-450 -> -7071 -> -0.7071)
*/
s16 okmath_sin(s16 deg)
{
	s16 result = 0;               // dec4
	u16 d = okmath_scaledeg(deg); // dec1

	// Between 0 and 90 degrees.
	if((d >= 0) && (d <= 900))
		result = sintable[d / 5];
	
	// Between 90 and 180 degrees.
	else if((d > 900) && (d <= 1800))
		result = sintable[(900 - (d - 900)) / 5];
	
	// Between 180 and 270 degrees.
	else if((d > 1800) && (d <= 2700))
		result = -sintable[(d - 1800) / 5];
	
	// Between 270 and 360 degrees.
	else if((d >= 2700) && (d <= 3600))
		result = -sintable[(3600 - d) / 5];
	
	return result;
}

s16 okmath_cos(s16 deg)
{
	s16 result = 0;               // dec4
	u16 d = okmath_scaledeg(deg); // dec1

	// Between 0 and 90 degrees.
	if((d >= 0) && (d <= 900))
		result = sintable[(900 - d) / 5];
	
	// Between 90 and 180 degrees.
	else if((d > 900) && (d <= 1800))
		result = -sintable[(d - 900) / 5];
	
	// Between 180 and 270 degrees.
	else if((d > 1800) && (d <= 2700))
		result = -sintable[(2700 - d) / 5];
	
	// Between 270 and 360 degrees.
	else if((d >= 2700) && (d <= 3600))
		result = sintable[(d - 2700) / 5];

	return result;
}

/*
	acos lookup table broken into the parts to create a higher accuracy near acos(1).
		0 to 0.9 is done in steps of 0.0079 rads. (1/127)
		0.9 to 0.99 is done in steps of 0.0008 rads. (0.01/127)
		0.99 to 1 is done in steps of 0.0002 rads. (0.01/64)
*/
static const u16 acostable[] = 
{
	255, 254, 252, 251, 250, 249, 247, 246, 245, 243,
	242, 241, 240, 238, 237, 236, 234, 233, 232, 231,
	229, 228, 227, 225, 224, 223, 221, 220, 219, 217,
	216, 215, 214, 212, 211, 210, 208, 207, 206, 204,
	203, 201, 200, 199, 197, 196, 195, 193, 192, 190,
	189, 188, 186, 185, 183, 182, 181, 179, 178, 176,
	175, 173, 172, 170, 169, 167, 166, 164, 163, 161,
	160, 158, 157, 155, 154, 152, 150, 149, 147, 146,
	144, 142, 141, 139, 137, 135, 134, 132, 130, 128,
	127, 125, 123, 121, 119, 117, 115, 113, 111, 109,
	107, 105, 103, 101,  98,  96,  94,  92,  89,  87,
	 84,  81,  79,  76,  73,  73,  73,  72,  72,  72,
	 71,  71,  71,  70,  70,  70,  70,  69,  69,  69,
	 68,  68,  68,  67,  67,  67,  66,  66,  66,  65,
	 65,  65,  64,  64,  64,  63,  63,  63,  62,  62,
	 62,  61,  61,  61,  60,  60,  59,  59,  59,  58,
	 58,  58,  57,  57,  57,  56,  56,  55,  55,  55,
	 54,  54,  53,  53,  53,  52,  52,  51,  51,  51,
	 50,  50,  49,  49,  48,  48,  47,  47,  47,  46,
	 46,  45,  45,  44,  44,  43,  43,  42,  42,  41,
	 41,  40,  40,  39,  39,  38,  37,  37,  36,  36,
	 35,  34,  34,  33,  33,  32,  31,  31,  30,  29,
	 28,  28,  27,  26,  25,  24,  23,  23,  23,  23,
	 22,  22,  22,  22,  21,  21,  21,  21,  20,  20,
	 20,  19,  19,  19,  19,  18,  18,  18,  17,  17,
	 17,  17,  16,  16,  16,  15,  15,  15,  14,  14,
	 13,  13,  13,  12,  12,  11,  11,  10,  10,   9,
	  9,   8,   7,   6,   6,   5,   3,   0
};

/*
	okmath_acos()
	s16 cosine - DEC4
*/
s16 okmath_acos(s16 cosine)
{
	s16 rads = 0;
	u08 negative;
	u16 abscosine;
	
	// Record the sign of cosine and store it's absolute value.
	if(cosine < 0)
	{
		negative = 1;
		abscosine = -cosine;
	}
	else
	{
		negative = 0;
		abscosine = cosine;
	}
	
	// Cosine between 0 and 0.9.
	if((cosine >= 0) && (cosine < 9000))
		rads = ((u32)acostable[abscosine/79] * 616) / DEC1;
	
	// Cosine between 0.9 and 0.99.
	else if ((cosine >= 9000) && (cosine < 9900))
		rads = ((u32)acostable[(abscosine-9000) / 8 + 114] * 616) / DEC1;
	
	// Cosine between 0.99 and 1.0.
	else if ((cosine >= 9900) && (cosine <= 10000))
		rads = ((u32)acostable[(abscosine - 9900) / 2 + 227] * 616) / DEC1;
	
	// Account for the negative sign if required.
	if(negative != 0)
		rads = 31416 - rads;
	
	return rads;
}

/*
	okmath_atan()
	s16 opp - DEC1
	s16 adj - DEC1
*/
s16 okmath_atan2(s16 opp, s16 adj)
{
	u32 hypt = okmath_sqrt((s32)adj * adj + (s32)opp * opp));
	s16 rads = okmath_acos(((s32)adj * DEC3) / hypt);
	
	if(y < 0)
		rads = -rads;
		
	return rads;
}
