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
#include "nordic_common.h"
#include "segger_wrapper.h"

static void _time_update(sKalmanExtDescr *descr, sKalmanExtFeed *feed) {

	float c_theta = - descr->ker.theta_p * descr->ker.s_theta * feed->dt_ms / 1000.;
	descr->ker.s_theta += descr->ker.theta_p * descr->ker.c_theta * feed->dt_ms / 1000.;
	descr->ker.c_theta += c_theta;

	float norm = descr->ker.c_theta*descr->ker.c_theta + descr->ker.s_theta*descr->ker.s_theta;
	descr->ker.s_theta /= my_sqrtf(norm);
	descr->ker.c_theta /= my_sqrtf(norm);
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

	// try to work with scalars only...

	// step 1: propagate
	_time_update(descr, feed);

//	descr->cov.p_mat = descr->cov.p_mat * feed->dt_ms * feed->dt_ms;

	// step 2: compute innovation
	// I = Z - H.X
	descr->innov.s_theta = descr->ker.s_theta - feed->acc[1] / 9.81;
	descr->innov.c_theta = descr->ker.c_theta - (feed->acc[0] - feed->gyr*feed->gyr*descr->innov.dx) / 9.81;

	descr->innov.theta_p = feed->gyr;

	if (feed->gyr > 0.0F) {
		descr->innov.dx = (feed->acc[0] - my_sqrtf(9.81 * 9.81 - feed->acc[1] * feed->acc[1]));
		descr->innov.dx /= (feed->gyr * feed->gyr);
	} else {
		descr->innov.dx = 0.0F;
	}

	if (descr->innov.dx > 0.0F) {
		descr->innov.theta_p_offset = feed->gyr;
		descr->innov.theta_p_offset += 1000 * atan2f(descr->ker.s_theta, descr->ker.c_theta) / feed->dt_ms;
		descr->innov.theta_p_offset -= 1000 * atan2f(descr->innov.s_theta, descr->innov.c_theta) / feed->dt_ms;
	}

	// compute gain
//	descr->cov.k_mat = descr->cov.p_mat / (descr->cov.p_mat + descr->cov.r_mat);

	// compute estimates
//	descr->ker.theta_p = descr->ker.theta_p - descr->ker.theta_p_offset + descr->cov.k_mat * descr->innov.theta_p;

	descr->ker.c_theta = 0.9 * descr->ker.c_theta + 0.1 * descr->innov.c_theta;
	descr->ker.s_theta = 0.9 * descr->ker.s_theta + 0.1 * descr->innov.s_theta;

	descr->ker.theta_p = 0.9 * descr->ker.theta_p + 0.1 * descr->innov.theta_p;

	descr->ker.dx = 0.95 * descr->ker.dx + 0.04 * descr->innov.dx;

	descr->ker.theta_p_offset = 0.95 * descr->ker.theta_p_offset + 0.05 * descr->innov.theta_p_offset;

	LOG_INFO("Estimated rot.: %f %f %f %f %f",
			descr->ker.c_theta,
			descr->ker.s_theta,
			descr->ker.theta_p,
			descr->ker.theta_p_offset,
			descr->ker.dx);
}
