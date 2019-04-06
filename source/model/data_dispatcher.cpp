/*
 * data_dispatcher.cpp
 *
 *  Created on: 4 apr. 2019
 *      Author: v.golle
 */


#include "gpio.h"
#include "data_dispatcher.h"
#include "math_wrapper.h"
#include "segger_wrapper.h"

#ifdef BLE_STACK_SUPPORT_REQD
#include "ble_api_base.h"
#endif

static float m_angle = 0;
static uint32_t m_cadence = 0;

void data_dispatcher_feed_gyro(float mdeg_s) {

	if (!isnormal(mdeg_s)) {
		LOG_ERROR("Illegal float");
		return;
	}

	m_cadence = (uint32_t)(fabsf(mdeg_s / 1000.) * 60. / 360.);

	// integrate angular speed over time (25Hz)
	float val = fabsf(mdeg_s / 1000.) / 25.;
	m_angle += val;

	if (m_angle > 360.) {
		m_angle = 0.0;

		LOG_WARNING("Full turn !");

		gpio_set(LED_1);

	} else {

		gpio_clear(LED_1);

	}

#ifdef BLE_STACK_SUPPORT_REQD
	ble_nus_log_cadence(m_cadence, 0);
#endif
}


bool data_dispatcher_get_batt_volt(uint32_t *batt_mv) {
	if (batt_mv) *batt_mv = 2200;
	return true;
}

bool data_dispatcher_get_cadence(uint32_t *cad) {
	if (cad) *cad = m_cadence;
	return true;
}

