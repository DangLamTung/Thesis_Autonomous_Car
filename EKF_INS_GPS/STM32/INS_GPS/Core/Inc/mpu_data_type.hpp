/*
 * mpu_data_type.hpp
 *
 *  Created on: Feb 10, 2020
 *      Author: tung
 */

#ifndef INC_MPU_DATA_TYPE_HPP_
#define INC_MPU_DATA_TYPE_HPP_
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DEC2RAD 0.01745329251

double Q[49]= {  0,0,0,0,0,0,0,
        0,0,0,0,0,0, 0,
         0,0,0,0,0,0,0,
          0,0,0,0,0,0,0,
		  0,0, 0 ,0 ,0.001 ,0, 0,
          0,0,0,0,0,0.001,0,
          0,0,0,0,0,0,0.001};
double P[49]= {1, 0, 0, 0, 0, 0, 0,
	          0,1,0, 0, 0, 0, 0,
	          0, 0,1,0, 0, 0, 0,
	          0, 0, 0,1,0, 0, 0,
	          0, 0, 0, 0,1,0, 0,
	          0, 0, 0, 0, 0,1,0,
	          0, 0, 0, 0, 0, 0,1};
double R_full[36]= {0.0001, 0, 0, 0, 0, 0,
					0, 0.0001, 0, 0, 0, 0,
					0, 0, 0.0001, 0, 0, 0,
					0, 0, 0, 0.0001, 0, 0,
					0, 0, 0, 0, 0.0001, 0,
					0, 0, 0, 0, 0, 0.0001};
double R[9]= {   0.001,0,0,
                 0,0.001,0,
                 0,0,0.001
                 };

#define MAX_PRECISION	(10)
static const double rounders[MAX_PRECISION + 1] =
{
	0.5,				// 0
	0.05,				// 1
	0.005,				// 2
	0.0005,				// 3
	0.00005,			// 4
	0.000005,			// 5
	0.0000005,			// 6
	0.00000005,			// 7
	0.000000005,		// 8
	0.0000000005,		// 9
	0.00000000005		// 10
};

char * ftoa(double f, char * buf, int precision)
{
	char * ptr = buf;
	char * p = ptr;
	char * p1;
	char c;
	long intPart;

	// check precision bounds
	if (precision > MAX_PRECISION)
		precision = MAX_PRECISION;

	// sign stuff
	if (f < 0)
	{
		f = -f;
		*ptr++ = '-';
	}

	if (precision < 0)  // negative precision == automatic precision guess
	{
		if (f < 1.0) precision = 6;
		else if (f < 10.0) precision = 5;
		else if (f < 100.0) precision = 4;
		else if (f < 1000.0) precision = 3;
		else if (f < 10000.0) precision = 2;
		else if (f < 100000.0) precision = 1;
		else precision = 0;
	}

	// round value according the precision
	if (precision)
		f += rounders[precision];

	// integer part...
	intPart = f;
	f -= intPart;

	if (!intPart)
		*ptr++ = '0';
	else
	{
		// save start pointer
		p = ptr;

		// convert (reverse order)
		while (intPart)
		{
			*p++ = '0' + intPart % 10;
			intPart /= 10;
		}

		// save end pos
		p1 = p;

		// reverse result
		while (p > ptr)
		{
			c = *--p;
			*p = *ptr;
			*ptr++ = c;
		}

		// restore end pos
		ptr = p1;
	}

	// decimal part
	if (precision)
	{
		// place decimal point
		*ptr++ = '.';

		// convert
		while (precision--)
		{
			f *= 10.0;
			c = f;
			*ptr++ = '0' + c;
			f -= c;
		}
	}

	// terminating zero
	*ptr = 0;

	return buf;
}
typedef struct
{
	float bGx;
	float bGy;
	float bGz;
	float bAx;
	float bAy;
	float bAz;
    float data[6];
} IMU_calib_data;

typedef struct
{
	float Gyro_x;
	float Gyro_y;
	float Gyro_z;
	float Acc_x;
	float Acc_y;
	float Acc_z;

} IMU_data;

typedef struct
{
	float Mag_x;
	float Mag_y;
	float Mag_z;

} MAG_data;

typedef struct
{
	float roll;
	float pitch;
	float yaw;

} EULER_angle;

typedef struct
{
	float q0;
	float q1;
	float q2;
	float q3;
} quaternion;

typedef struct
{
	bool IMU = true;
	bool magnet = true;
	bool baro = true;
} AHRS_con;


float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
#define  PI 3.141592654
#define  RAD2DEC 57.29577951

#endif /* INC_MPU_DATA_TYPE_HPP_ */
