/*
 * unit_testing.cpp
 *
 *  Created on: 3 nov. 2018
 *      Author: Vincent
 */

#include <cmath>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "Model_tdd.h"
#include "UDMatrix.h"
#include "kalman_ext.h"
#include "sine_fitter.h"
#include "fram.h"
#include "utils.h"

#include "order1_filter.h"

#define TEST_FILTRE_NB    15

#define TEST_ROLLOF_NB    48759

bool test_kalman_ext(void) {

	LOG_INFO("Testing Kalman...");

//	UDMatrix mat1(3, 3);
//	mat1.m_data[0][0] = 5;
//	mat1.m_data[0][1] = 7;
//	mat1.m_data[0][2] = 9;
//	mat1.m_data[1][0] = 4;
//	mat1.m_data[1][1] = 3;
//	mat1.m_data[1][2] = 8;
//	mat1.m_data[2][0] = 7;
//	mat1.m_data[2][1] = 5;
//	mat1.m_data[2][2] = 6;
//
//	mat1.print();
//
//	UDMatrix matx(3, 6);
//	matx.unity();
//
//	UDMatrix mat2(3, 3);
//	mat2 = matx.transpose();
//
//	mat2.print();
//
//	UDMatrix mat3(3, 3);
//	mat3 = mat1.invert();
//	mat3.print();
//
//	LOG_INFO("Invert:");
//	mat2 = mat1 * mat3;
//	mat2.print();

	float sampling = 0.5;
	uint16_t numOfData = 30;

	sKalmanDescr descr;
	sKalmanExtFeed feed;

	descr.ker.ker_dim = 4;
	descr.ker.obs_dim = 5;

	feed.dt = sampling;

	kalman_ext_init(&descr);

	feed.matZ.resize(5, 1);

	descr.ker.matA.unity();
	descr.ker.matA.set(1, 0, 1);
	descr.ker.matA.print();

	// TODO set H
//	descr.ker_ext.matH

	// set Q
	descr.ker.matQ.unity(1 / 20.);

	// set P
	descr.ker.matP.ones(900);

	// set R
	descr.ker.matR.unity(0.1);

	float val;
	for(int i=0;i < numOfData;i++)
	{
		descr.ker.matC.set(0, 0, 1);

		descr.ker.matA.set(0, 2, feed.dt);
		descr.ker.matA.set(1, 2, feed.dt);

		val = 22 +  5 * i;
		feed.matZ.ones(0.0);
		feed.matZ.set(0, 0, val);

		kalman_ext_feed(&descr, &feed);

		UDMatrix res;
		res = descr.ker.matX;
		res.print();
	}

	LOG_INFO("Simulated pos: %f", val);

	exit(0);

	return true;
}

bool test_kalman_lin(void) {

	LOG_INFO("Testing Kalman...");

	uint16_t numOfData = 30;
	float omega = 2 * M_PI; // speed in rad / s
	float sampling = 1; // in seconds

	sKalmanDescr descr;
	sKalmanExtFeed feed;

	descr.ker.ker_dim = 2;
	descr.ker.obs_dim = 1;

	feed.dt = sampling;

	kalman_lin_init(&descr);

	feed.matZ.resize(1, 1);

	descr.ker.matA.unity();

	descr.ker.matA.print();

	descr.ker.matC.set(0, 0, 1);

	// set Q
	descr.ker.matQ.unity(1 / 20.);

	// set P
	descr.ker.matP.ones(900);

	// set R
	descr.ker.matR.unity(0.1);

	float val;
	for(int i=0;i < numOfData;i++)
	{
		val = 22 +  5 * i;

		descr.ker.matA.set(0, 1, feed.dt);

		feed.matZ.ones(0.0);
		feed.matZ.set(0, 0, val);

		kalman_lin_feed(&descr, &feed);

		UDMatrix res;
		res = descr.ker.matX;
		res.print();
	}

	LOG_INFO("Simulated pos: %f", val);

	exit(0);

	return true;
}

bool test_sine_fitting(void) {

	LOG_INFO("Testing sine fitting...");

	uint16_t numOfData = 30;
	float * dataz = new float[numOfData];
	float omega = 2. * M_PI; // speed in rad / s
	float sampling = 0.1; // in seconds

	for(int i=0;i < numOfData;i++)
	{
	  dataz[i] = 125 + 7 * sin(omega * sampling * i + 0.2);
	}

	sSineFitterOuput output;
	sine_fitter_compute(dataz, omega, sampling, numOfData, &output);

	LOG_INFO("Res.: %f %f %f",
			output.alpha,
			output.beta,
			output.phi);

	LOG_INFO("Fitting OK");

	return true;
}

bool test_fram(void) {

	LOG_INFO("Testing FRAM...");

	fram_init_sensor();

	if (!u_settings.isConfigValid())
		return false;

	if (!u_settings.resetConfig())
		return false;

	if (!u_settings.isConfigValid())
		return false;

	LOG_INFO("FRAM OK");

	return true;
}

bool test_power_zone(void) {

	PowerZone p_zones;
	uint32_t timestamp = 0;

	LOG_INFO("Testing suffer score...");

	for (int i=0; i < 116; i++) p_zones.addPowerData(110, (timestamp++)*1000);
	for (int i=0; i < 158; i++) p_zones.addPowerData(175, (timestamp++)*1000);
	for (int i=0; i < 1831; i++) p_zones.addPowerData(206, (timestamp++)*1000);
	for (int i=0; i < 812; i++) p_zones.addPowerData(242, (timestamp++)*1000);
	for (int i=0; i < 330; i++) p_zones.addPowerData(401, (timestamp++)*1000);

	LOG_INFO("Time spent in PZ %u", p_zones.getTimeTotal());

	if (p_zones.getTimeTotal() < 4) return false;

	if (fabsf(p_zones.getTimeMax() - 1831) > 2) return false;

	return true;
}

