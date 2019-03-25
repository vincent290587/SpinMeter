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
#include "fram.h"
#include "app_timer.h"
#include "Model.h"


APP_TIMER_DEF(m_timer);


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

	// TODO Init sensors configuration
	bmg250_wrapper_init();

	lis2dw12_wrapper_init();

	// post-init steps
	_i2c_scheduling_sensors_post_init();
}

/**
 *
 * @param p_context
 */
static void timer_handler(void * p_context)
{
	W_SYSVIEW_RecordEnterISR();

	// TODO

    W_SYSVIEW_RecordExitISR();
}

/**
 *
 */
void i2c_scheduling_init(void) {

	_i2c_scheduling_sensors_init();

	delay_ms(3);

	ret_code_t err_code;
	err_code = app_timer_create(&m_timer, APP_TIMER_MODE_REPEATED, timer_handler);
	APP_ERROR_CHECK(err_code);

//	err_code = app_timer_start(m_timer, APP_TIMER_TICKS(I2C_SCHEDULING_PERIOD_MS), NULL);
//	APP_ERROR_CHECK(err_code);

}

void i2c_scheduling_tasks(void) {

	if (bmg250_wrapper_is_updated()) {
		bmg250_wrapper_sensor_refresh();
	}

}
