/*
 * data_dispatcher.cpp
 *
 *  Created on: 4 apr. 2019
 *      Author: v.golle
 */


#include "nrf_gpio.h"
#include "boards.h"
#include "data_dispatcher.h"
#include "math_wrapper.h"
#include "segger_wrapper.h"


static float m_turns = 0;
static uint32_t m_cadence = 0;

void data_dispatcher_feed_gyro(int32_t ddeg_s) {

	m_cadence = abs(ddeg_s) * 60 / 1800;

	int val = abs(ddeg_s) * 10 / 25;
	m_turns += (float)val;

	if (m_turns > 360.) {
		m_turns = 0.0;

		LOG_WARNING("Full turn !");

		nrf_gpio_pin_set(LED_1);

	} else {

		nrf_gpio_pin_clear(LED_1);

	}

}


bool data_dispatcher_get_batt_volt(uint32_t *batt_mv) {
	if (batt_mv) *batt_mv = 2200;
	return true;
}

bool data_dispatcher_get_cadence(uint32_t *cad) {
	if (cad) *cad = m_cadence;
	return true;
}

