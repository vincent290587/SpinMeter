/*
 * kalman_ext.h
 *
 *  Created on: 26 mrt. 2019
 *      Author: v.golle
 */

#ifndef LIBRARIES_KALMAN_KALMAN_EXT_H_
#define LIBRARIES_KALMAN_KALMAN_EXT_H_

#include <stdint.h>
#include "UDMatrix.h"

/**
 * Input used to estimate the rotation rate only
 */
typedef struct {
	float acc[3];
	float gyr;
	uint32_t dt_ms;
} sKalmanExtFeed;

typedef struct {
	UDMatrix matC;
	UDMatrix matE;
	UDMatrix matK;
	UDMatrix matX;
	UDMatrix matXmi;
	UDMatrix matPmi;
	UDMatrix matP;
	UDMatrix matQ;
	float c_theta;
	float s_theta;
	float theta_p;
	float theta_p_offset;
} sKalmanExtKernel;

typedef struct {
	float p_mat;
	float q_mat;
	float r_mat;
	float k_mat;
} sKalmanExtCovs;

typedef struct {
	sKalmanExtKernel ker;
	sKalmanExtKernel innov;
	sKalmanExtCovs cov;
	uint8_t is_init;
} sKalmanExtDescr;


#ifdef	__cplusplus
extern "C" {
#endif


void kalman_ext_init(sKalmanExtDescr *descr);

void kalman_ext_feed(sKalmanExtDescr *descr, sKalmanExtFeed *feed);


#ifdef	__cplusplus
}
#endif


#endif /* LIBRARIES_KALMAN_KALMAN_EXT_H_ */
