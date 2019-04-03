/*
 * fdc1004_wrapper.c
 *
 *  Created on: 28 mrt. 2019
 *      Author: v.golle
 */

#include <stdbool.h>
#include "FDC1004.h"
#include "fdc1004_wrapper.h"
#include "segger_wrapper.h"

void fdc1004_meas_trigger(void) {

	FDC1004_read_raw_measurement(0, NULL);

	// set trigger
	sChannelTrigger trigger;
	trigger.val = 0;

	trigger.bitfield.meas_rate = eConfRegRate400SPS;

	// enable meas on ch1
	trigger.bitfield.ch1_m_en = 1;
	FDC1004_trigger_measurement(&trigger);
}

void fdc1004_wrapper_init(void)
{
	uint8_t ret = FDC1004_init();
	// init module
	if (!ret) {

		// configure measurement
		sChannelMeasurement ch_meas;
		ch_meas.val = 0;

		ch_meas.bitfield.p_channel = eCHACIN1;
		ch_meas.bitfield.n_channel = eCHACIN2;

		FDC1004_configure_differential_measurement(0, &ch_meas);

		// set trigger
		sChannelTrigger trigger;
		trigger.val = 0;

		trigger.bitfield.meas_rate = eConfRegRate400SPS;

		// enable meas on ch1
		trigger.bitfield.ch1_m_en = 1;
		FDC1004_trigger_measurement(&trigger);

		LOG_WARNING("FDC1004 Init success");

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

		FDC1004_clear_updated();
	}


}
