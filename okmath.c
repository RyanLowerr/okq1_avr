
#include <avr/io.h>

#include "okmath.h"

void vector_normalize(VECTOR *vi, VECTOR *vo)
{
	;
}

void vector_magnitude(VECTOR *v)
{
	;
}

void vector_rotate(VECTOR *vi, VECTOR *vo, float roll, float pitch, float yaw)
{
	;
}

static u16 scaledeg(s16 deg);

// Sin lookup table. 0 to 90 degrees with 0.5 degree steps. stored as u16 dec4
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
	The functions lookupsin and lookupcos use sintable to determine the sin or cos value of the input angle.
	Both functions assume input deg is an angle in degrees represented as a s16 dec1. (-450 = -45.0 deg)
	Both functions return a value between -1 and 1 as a s16 dec4. (-450 -> -7071 -> -0.7071)
*/
s16 lookupsin(s16 deg)
{
	s16 result = 0;        // dec4
	u16 d = scaledeg(deg); // dec1

	// Between 0 and 90 degrees.
	if((d >= 0) && (d <= 900))
		result = sintable[d / 5];
	
	// Between 90 and 180 degrees.
	else if((d > 900) && (d <= 1800))
		result = sintable[900 - (d - 900) / 5];
	
	// Between 180 and 270 degrees.
	else if((d > 1800) && (d <= 2700))
		result = sintable[(d - 1800) / 5];
	
	// Between 270 and 360 degrees.
	else if((d >= 2700) && (d <= 3600))
		result = sintable[(3600 - d) / 5];
	
	return result;
}

s16 lookupcos(s16 deg)
{
	s16 result = 0;        // dec4
	u16 d = scaledeg(deg); // dec1

	// Between 0 and 90 degrees.
	if((d >= 0) && (d <= 900))
		result = sintable[(900 - d) / 5];
	
	// Between 90 and 180 degrees.
	else if((d > 900) && (d <= 1800))
		result = sintable[(d - 900) / 5];
	
	// Between 180 and 270 degrees.
	else if((d > 1800) && (d <= 2700))
		result = sintable[(2700 - d) / 5];
	
	// Between 270 and 360 degrees.
	else if((d >= 2700) && (d <= 3600))
		result = sintable[(d - 2700) / 5];

	return result;
}

/*
	The function scaledeg scales input angle to a value between 0 and 360 degrees. 
	Input deg is an angle in degrees represented as a s16 dec1. (-450 = -45.0 degrees)
	Returns scaled input deg as a u16 dec1. (-450 -> 3150 = 315.0 degrees)
*/
static u16 scaledeg(s16 deg)
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
