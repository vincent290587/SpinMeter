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
 *      https://dsp.stackexchange.com/questions/36135/tracking-a-sine-wave-with-kalman-filter-how-to-account-for-offset-dc-signal
 */

#include "UDMatrix.h"
#include "kalman_ext.h"
#include "math_wrapper.h"
#include "nordic_common.h"
#include "segger_wrapper.h"

static void _time_update(sKalmanExtDescr *descr, sKalmanExtFeed *feed) {

}

void kalman_ext_init(sKalmanExtDescr *descr) {
	descr->cov.r_mat = 0.1;
	descr->cov.p_mat = 100;

	descr->is_init = 0;
}

void kalman_ext_feed(sKalmanExtDescr *descr, sKalmanExtFeed *feed) {

	ASSERT(descr);
	ASSERT(feed);

	// limit accelerometer vector
	feed->acc[1] = MIN(feed->acc[1], 9.81);
	feed->acc[1] = MAX(feed->acc[1], -9.81);

	if (!descr->is_init) {
		descr->ker.s_theta = feed->acc[1] / 9.81;
		descr->ker.c_theta = my_sqrtf(1 - descr->ker.s_theta*descr->ker.s_theta);
		descr->ker.theta_p = feed->gyr;
		descr->ker.theta_p_offset = 0.0;
		descr->ker.dx = 0.1;

		descr->is_init = 1;
	}
}
