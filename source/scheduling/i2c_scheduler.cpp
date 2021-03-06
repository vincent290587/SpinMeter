/*
 * i2c_scheduler.c
 *
 *  Created on: 10 déc. 2017
 *      Author: Vincent
 */

#include "boards.h"
#include "helper.h"
#include "i2c.h"
#include "gpio.h"
#include "i2c_scheduler.h"
#include "segger_wrapper.h"
#include "parameters.h"
#include "millis.h"
#include "bmg250_wrapper.h"
#include "lis2dw12_wrapper.h"
#include "fdc1004_wrapper.h"
#include "fram.h"
#include "Model.h"


static void _int1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

	W_SYSVIEW_RecordEnterISR();

	// clear trigger
	gpio_toggle(LED_1);

    // schedule sensor reading
	bmg250_wrapper_schedule_sensor();

	// trigger a measurement in the lis2dw & bmg250
	lis2dw12_meas_trigger();

//	fdc1004_meas_trigger();

	W_SYSVIEW_RecordExitISR();
}

/**
 *
 */
static void _i2c_scheduling_sensors_post_init(void) {

	// configure GPIOTE
	nrfx_gpiote_in_config_t in_config;
	in_config.is_watcher = true;
	in_config.hi_accuracy = true;
	in_config.skip_gpio_setup = false;
	in_config.pull = NRF_GPIO_PIN_PULLDOWN;
	in_config.sense = NRF_GPIOTE_POLARITY_LOTOHI;

	ret_code_t err_code = nrfx_gpiote_in_init(GYR_INT1, &in_config, _int1_handler);
	APP_ERROR_CHECK(err_code);

	nrfx_gpiote_in_event_enable(GYR_INT1, true);
}


/**
 *
 */
static void _i2c_scheduling_sensors_init() {

	// Init sensors configuration

//	fdc1004_wrapper_init();

	lis2dw12_wrapper_init();

	bmg250_wrapper_init();
}

/**
 *
 */
void i2c_scheduling_init(void) {

	_i2c_scheduling_sensors_init();

	LOG_WARNING("Sensors initialized");

	// post-init steps
	_i2c_scheduling_sensors_post_init();

}

/**
 *
 */
void i2c_scheduling_uninit(void) {

	i2c_uninit();

	nrfx_gpiote_in_uninit(GYR_INT1);

}

void i2c_scheduling_tasks(void) {

	if (bmg250_wrapper_is_updated()) {
		sysview_task_transfer(TASK0);
		bmg250_wrapper_sensor_refresh();
	}
	if (lis2dw12_wrapper_is_updated()) {
		sysview_task_transfer(TASK1);
		lis2dw12_wrapper_sensor_refresh();
		// TODO trigger kalman
	}

	fdc1004_wrapper_sensor_tasks();

}
