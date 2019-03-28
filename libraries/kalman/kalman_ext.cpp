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

	descr->ker.matA.zeros();
	descr->ker.matA.m_data[0][1] = 1;
	descr->ker.matA.m_data[1][0] = - feed->gyr * feed->gyr;

	descr->ker.matXmi = descr->ker.matA * descr->ker.matK;
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

		// size matrixes
		descr->ker.matA.resize(3, 3);
		descr->ker.matE.resize(3, 3);
		descr->ker.matP.resize(3, 3);
		descr->ker.matPmi.resize(3, 3);
		descr->ker.matK.resize(3, 1);
		descr->ker.matQ.resize(3, 3);
		descr->ker.matX.resize(3, 1);
		descr->ker.matXmi.resize(3, 1);

		descr->ker.matC.resize(1, 3);
		descr->ker.matC.m_data[0][0] = 1;
		descr->ker.matC.m_data[0][1] = 0;
		descr->ker.matC.m_data[0][2] = 1;

		// TODO set Q
		descr->ker.matQ.m_data[0][2] = 1;
		descr->ker.matQ.m_data[1][1] = 0;
		descr->ker.matQ.m_data[2][0] = 1;

		descr->is_init = 1;

	}

	_time_update(descr, feed);

	// Measurement update
	UDMatrix matAt(3, 3);
	matAt = descr->ker.matA.transpose();

	// project covariance
	descr->ker.matPmi = descr->ker.matA * descr->ker.matP;
	descr->ker.matPmi = descr->ker.matPmi * matAt;
	descr->ker.matPmi = descr->ker.matPmi + descr->ker.matQ;

	// y - Cx_est
	UDMatrix matI(1, 1);
	matI = descr->ker.matC * descr->ker.matXmi;
	float innov = matI.m_data[0][0] - feed->gyr;

	// update kalman gain
	UDMatrix matCt(3, 1);
	matCt = descr->ker.matC.transpose();
	descr->ker.matK = descr->ker.matPmi * matCt;
	UDMatrix matCp(1, 1);
	matCp = descr->ker.matC * descr->ker.matK;
	matCp.m_data[0][0] += descr->ker.r;
	descr->ker.matK.div(matCp.m_data[0][0]);

	// update estimate
	UDMatrix matKI;
	matKI = descr->ker.matK;
	matKI.mul(innov);
	descr->ker.matX = descr->ker.matA * descr->ker.matXmi;
	descr->ker.matX = descr->ker.matX + matKI;

	// update covariance
	UDMatrix matTmp(3, 3);
	matKI.resize(3, 3);
	matKI.unity();
	matTmp = descr->ker.matK * descr->ker.matC;
	matKI = matKI - matTmp;
	descr->ker.matP = matKI * descr->ker.matPmi;
}
