/*
 * kalman_ext.h
 *
 *  Created on: 26 mrt. 2019
 *      Author: v.golle
 */

#ifndef LIBRARIES_KALMAN_KALMAN_EXT_H_
#define LIBRARIES_KALMAN_KALMAN_EXT_H_

/**
 * Input used to estimate the rotation rate only
 */
typedef struct {
	float acc[3];
	float gyr;
} sKalmanExtFeed;


#endif /* LIBRARIES_KALMAN_KALMAN_EXT_H_ */
