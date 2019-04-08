/*
 * data_dispatcher.cpp
 *
 *  Created on: 4 apr. 2019
 *      Author: v.golle
 */


#include "gpio.h"
#include "millis.h"
#include "Model.h"
#include "data_dispatcher.h"
#include "math_wrapper.h"
#include "segger_wrapper.h"

#ifdef BLE_STACK_SUPPORT_REQD
#include "ble_api_base.h"
#endif

static float m_angle = 0;
static uint32_t m_cadence = 0;

static uint32_t m_static_nb = 0;

static void _check_is_moving(float mdeg_s) {

	if (fabsf(mdeg_s) < 500 &&
			m_static_nb++ > 2 * 60 * 25) {
		LOG_ERROR("Inactivity timeout: going to shutdown");

#ifdef BLE_STACK_SUPPORT_REQD
		ble_nus_log_text("Inactivity timeout: going to shutdown");
		ble_nus_tasks();
		delay_ms(10);
#endif

		app_shutdown();
		m_static_nb = 0;
	} else {
		m_static_nb = 0;
	}

}

void data_dispatcher_feed_gyro(float mdeg_s) {

	if (!isnormal(mdeg_s)) {
		LOG_ERROR("Illegal float");
		return;
	}

	_check_is_moving(mdeg_s);

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
	// log cadence through BLE every second
	static int nb_ind = 25;
	if (nb_ind-- == 0) {
		ble_nus_log_cadence(m_cadence, 0);
		nb_ind = 25;
	}
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

