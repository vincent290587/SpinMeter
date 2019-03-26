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
#include "math_wrapper.h"
#include "segger_wrapper.h"

static void _time_update(sKalmanExtDescr *descr, sKalmanExtFeed *feed) {

	descr->ker.theta = descr->ker.theta_p * feed->dt_ms / 1000.;

}

void kalman_ext_init(sKalmanExtDescr *descr) {
	descr->cov.r_mat = 0.1;
	descr->cov.p_mat = 100;

	descr->is_init = 0;
}

void kalman_ext_feed(sKalmanExtDescr *descr, sKalmanExtFeed *feed) {

	ASSERT(descr);
	ASSERT(feed);

	if (!descr->is_init) {
		descr->ker.theta = 0.0;
		descr->ker.theta_p = feed->gyr;
		descr->ker.theta_p_offset = 0.0;
		descr->ker.dx = 0.1;

		descr->is_init = 1;
	}

	// try to work with scalars only...

	// step 1: propagate
	_time_update(descr, feed);

//	descr->cov.p_mat = descr->cov.p_mat * feed->dt_ms * feed->dt_ms;

	// step 2: compute innovation
	// I = Z - H.X
	descr->innov.theta = asinf(feed->acc[1] / 9.81);

	descr->innov.theta_p = feed->gyr;

	if (feed->gyr > 0.0F) {
		descr->innov.dx = (feed->acc[0] - my_sqrtf(9.81 * 9.81 - feed->acc[1] * feed->acc[1]));
		descr->innov.dx /= (feed->gyr * feed->gyr);
	} else {
		descr->innov.dx = 0.0F;
	}

	if (descr->innov.dx > 0.0F) {
		descr->innov.theta_p_offset = feed->gyr;
		descr->innov.theta_p_offset -= (feed->acc[0] + cosf(descr->innov.theta) * 9.81) / descr->innov.dx;
	} else {
		descr->innov.theta_p_offset = 0;
	}

	// compute gain
//	descr->cov.k_mat = descr->cov.p_mat / (descr->cov.p_mat + descr->cov.r_mat);

	// compute estimates
//	descr->ker.theta_p = descr->ker.theta_p - descr->ker.theta_p_offset + descr->cov.k_mat * descr->innov.theta_p;
	descr->ker.theta = 0.9 * descr->ker.theta + 0.1 * descr->innov.theta;

	descr->ker.theta_p = 0.9 * descr->ker.theta_p + 0.1 * descr->innov.theta_p;

	descr->ker.dx = 0.95 * descr->ker.dx + 0.04 * descr->innov.dx;

	descr->ker.theta_p_offset = 0.95 * descr->ker.theta_p_offset + 0.05 * descr->innov.theta_p_offset;
}
