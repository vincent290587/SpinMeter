/*
 * data_dispatcher.cpp
 *
 *  Created on: 4 apr. 2019
 *      Author: v.golle
 */


#include "data_dispatcher.h"
#include "math_wrapper.h"
#include "segger_wrapper.h"

static uint32_t m_cadence = 0;

void data_dispatcher_feed_gyro(int32_t ddeg_s) {

	m_cadence = abs(ddeg_s) * 60 / 1800;

}


bool data_dispatcher_get_batt_volt(uint32_t *batt_mv) {
	if (batt_mv) *batt_mv = 2200;
	return true;
}

bool data_dispatcher_get_cadence(uint32_t *cad) {
	if (cad) *cad = m_cadence;
	return true;
}

