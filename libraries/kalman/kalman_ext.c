/*
 * kalman_ext.h
 *
 *  Created on: 26 mrt. 2019
 *      Author: v.golle
 *
 *      http://bilgin.esme.org/BitsAndBytes/KalmanFilterforDummies
 *
 *      https://stackoverflow.com/questions/41754085/tracking-a-robot-in-circular-motion-using-kalman-filter
 *
 */


#include "kalman_ext.h"


typedef struct {
	float theta;
	float theta_p;
	float theta_p_offset;
} sKalmanExtKernel;

