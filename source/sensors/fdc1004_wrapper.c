/*
 * fdc1004_wrapper.c
 *
 *  Created on: 28 mrt. 2019
 *      Author: v.golle
 */

#include <stdbool.h>
#include "FDC1004.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"
#include "fdc1004_wrapper.h"
#include "segger_wrapper.h"

static bool m_is_init = false;

void fdc1004_meas_trigger(void) {

	if (!m_is_init) return;

	FDC1004_read_raw_measurement(0, NULL);

}

void fdc1004_wrapper_init(void)
{

	// reset chip
	sChannelTrigger trigger;
	trigger.val = 0;
	trigger.bitfield.rst = 1;

	uint8_t ret = 0;
	do {
		FDC1004_trigger_measurement(&trigger);

		nrf_delay_ms(5);

		ret = FDC1004_init();
	} while (ret);

	// init module
	if (!ret) {

		// configure measurement
		sChannelMeasurement ch_meas;
		ch_meas.val = 0;

//		ch_meas.bitfield.p_channel = eCHACIN1;
//		ch_meas.bitfield.n_channel = eCHBCIN2;
//		FDC1004_configure_differential_measurement(0, &ch_meas);

//		ch_meas.bitfield.p_channel = eCHACIN1;
//		ch_meas.bitfield.n_channel = eCHBCDAC;
//		ch_meas.bitfield.capdac = 0b11111;
//		FDC1004_configure_single_measurement(0, &ch_meas);

		ch_meas.bitfield.p_channel = eCHACIN2;
		ch_meas.bitfield.n_channel = eCHBDis;
		FDC1004_configure_single_measurement(0, &ch_meas);

		// set trigger
		sChannelTrigger trigger;
		trigger.val = 0;

		trigger.bitfield.meas_rate = eConfRegRate400SPS;

		// enable meas
		trigger.bitfield.ch1_m_en = 1;
		FDC1004_trigger_measurement(&trigger);

		LOG_WARNING("FDC1004 Init success");

		m_is_init = true;

	} else {
		LOG_ERROR("FDC 1004 init problem: %u", ret);
	}
}

void fdc1004_wrapper_sensor_tasks(void) {

	if (FDC1004_is_updated()) {

		// convert
		float delta_cap;
		FDC1004_read_measurement(0, &delta_cap);

		// TODO process
		LOG_WARNING("Raw val in fF: %d", (int32_t)(delta_cap * 1000));

		FDC1004_clear_updated();
	}


}
