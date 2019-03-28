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

#include "kalman_ext.h"
#include "math_wrapper.h"
#include "nordic_common.h"
#include "segger_wrapper.h"

static void _time_update(sKalmanExtDescr *descr, sKalmanExtFeed *feed) {

	UDMatrix matA(3, 3);
	matA.m_data[0][1] = 1;
	matA.m_data[1][0] = - feed->gyr * feed->gyr;

	descr->ker.matXmi = matA * descr->ker.matK;
}

void kalman_ext_init(sKalmanExtDescr *descr) {

	ASSERT(descr);
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

		// TODO size matrixes
		descr->ker.matE.resize(3, 3);
		descr->ker.matP.resize(3, 3);
		descr->ker.matK.resize(3, 3);
		descr->ker.matQ.resize(3, 3);
		descr->ker.matX.resize(3, 1);
		descr->ker.matXmi.resize(3, 1);

		descr->ker.matC.resize(1, 3);
		descr->ker.matC.m_data[0][0] = 1;
		descr->ker.matC.m_data[0][1] = 0;
		descr->ker.matC.m_data[0][2] = 1;

		descr->is_init = 1;

	} else {

		_time_update(descr, feed);

	}

	// Measurement update
	UDMatrix matA(3, 3);
	UDMatrix matAt(3, 3);
	matA = (descr->ker.matC * descr->ker.matXmi);
	matAt = matA.transpose();

	// project covariance
	descr->ker.matPmi = matA * descr->ker.matP;
	descr->ker.matPmi = descr->ker.matPmi * matAt;
	descr->ker.matPmi = descr->ker.matPmi + descr->ker.matQ;

	// y - Cx_est
	UDMatrix matI(1, 1);
	matI = descr->ker.matC * descr->ker.matXmi;
	float innov = matI.m_data[0][0] - feed->gyr;

	// TODO update kalman gain
	UDMatrix matCt(3, 1);
	matCt = descr->ker.matC.transpose();
	descr->ker.matK = descr->ker.matPmi * matCt;

	// update estimate
	UDMatrix matKI(3, 3);
	matKI.unity(innov);
	matKI = descr->ker.matK * matKI;
	descr->ker.matX = matA * descr->ker.matXmi;
	descr->ker.matX = descr->ker.matX + matKI;

	// update covariance
	matKI.unity();
	matA = descr->ker.matK * descr->ker.matC;
	matKI = matKI - matA;
	descr->ker.matP = matKI - descr->ker.matPmi;
}
