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

	descr->ker.matXmi = descr->ker.matA * descr->ker.matX;
}

void kalman_ext_init(sKalmanExtDescr *descr) {

	ASSERT(descr);
	descr->is_init = 0;
}

#define OBS_DIM    2

void kalman_ext_feed(sKalmanExtDescr *descr, sKalmanExtFeed *feed) {

	ASSERT(descr);
	ASSERT(feed);

	if (!descr->is_init) {

		// size matrixes
		descr->ker.matA.resize(3, 3);
		descr->ker.matE.resize(3, 3);
		descr->ker.matP.resize(3, 3);
		descr->ker.matPmi.resize(3, 3);
		descr->ker.matQ.resize(3, 3);
		descr->ker.matR.resize(OBS_DIM, OBS_DIM);

		descr->ker.matXmi.resize(3, 1);

		descr->ker.matK.resize(3, OBS_DIM);

		// set X
		descr->ker.matX.resize(3, 1);
		descr->ker.matX.m_data[0][0] = feed->acc[1];
		descr->ker.matX.m_data[1][0] = feed->gyr;

		// set C
		descr->ker.matC.resize(OBS_DIM, 3);
		descr->ker.matC.m_data[0][1] = 1;
		descr->ker.matC.m_data[1][0] = 1;

		// TODO set Q
		descr->ker.matQ.ones(1 / 20.);
//		descr->ker.matQ.transpose();

		// TODO set P
		descr->ker.matP.ones(900);

		// TODO set R
		descr->ker.matR.ones(0.1);

		descr->is_init = 1;

	}

	_time_update(descr, feed);

	// Measurement update
	UDMatrix matAt;
	matAt = descr->ker.matA.transpose();

	// project covariance
	descr->ker.matPmi = descr->ker.matA * descr->ker.matP;
	descr->ker.matPmi = descr->ker.matPmi * matAt;
	descr->ker.matPmi = descr->ker.matPmi + descr->ker.matQ;

	// update kalman gain
	UDMatrix matCt;
	matCt = descr->ker.matC.transpose();
	descr->ker.matK = descr->ker.matPmi * matCt;
	UDMatrix matCp;
	matCp = descr->ker.matC * descr->ker.matK;
	matCp = matCp + descr->ker.matR;
	UDMatrix matCpi;
	matCpi = matCp.invert();
	descr->ker.matK = descr->ker.matK * matCpi;


	// y - Cx_est
	UDMatrix matI;
	matI = descr->ker.matC * descr->ker.matXmi;
	UDMatrix innov(2, 1);
	innov.m_data[0][0] = feed->gyr;
	innov.m_data[1][0] = asinf(feed->acc[1] / 9.81);

	innov.print();

	// update estimate
	UDMatrix matKI;
	matKI = descr->ker.matK * innov;
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
