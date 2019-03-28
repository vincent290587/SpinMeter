/*
 * i2c_scheduler.c
 *
 *  Created on: 10 déc. 2017
 *      Author: Vincent
 */

#include "i2c.h"
#include "i2c_scheduler.h"
#include "segger_wrapper.h"
#include "parameters.h"
#include "millis.h"
#include "bmg250_wrapper.h"
#include "lis2dw12_wrapper.h"
#include "fdc1004_wrapper.h"
#include "fram.h"
#include "Model.h"


/**
 *
 */
static void _i2c_scheduling_sensors_post_init(void) {

	LOG_WARNING("Sensors initialized");
}


/**
 *
 */
static void _i2c_scheduling_sensors_init() {

	// Init sensors configuration
	bmg250_wrapper_init();

	lis2dw12_wrapper_init();

	fdc1004_wrapper_init();
}

/**
 *
 */
void i2c_scheduling_init(void) {

	_i2c_scheduling_sensors_init();

	// post-init steps
	_i2c_scheduling_sensors_post_init();

}

void i2c_scheduling_tasks(void) {

	if (bmg250_wrapper_is_updated()) {
		// trigger a measurement in the lis2dw & bmg250
		lis2dw12_meas_trigger();
		fdc1004_meas_trigger();
		bmg250_wrapper_sensor_refresh();
	}
	if (lis2dw12_wrapper_is_updated()) {
		lis2dw12_wrapper_sensor_refresh();
		// TODO trigger kalman
	}

	fdc1004_wrapper_sensor_tasks();

}
