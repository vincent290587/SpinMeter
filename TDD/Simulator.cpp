/*
 * Simulator.cpp
 *
 *  Created on: 18 sept. 2018
 *      Author: Vincent
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "millis.h"
#include "Model_tdd.h"
#include "Simulator.h"
#include "segger_wrapper.h"
#include "assert_wrapper.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/


void simulator_init(void) {

	m_app_error.hf_desc.crc = SYSTEM_DESCR_POS_CRC;
	m_app_error.hf_desc.stck.pc = 0x567896;

	m_app_error.special = SYSTEM_DESCR_POS_CRC;

	m_app_error.err_desc.crc = SYSTEM_DESCR_POS_CRC;
	snprintf(m_app_error.err_desc._buffer,
			sizeof(m_app_error.err_desc._buffer),
			"Error 0x123456 in file /mnt/e/Nordic/Projects/Perso/stravaV10/TDD/Simulator.cpp:48");

}

void simulator_tasks(void) {

	if (millis() < 5000) {
		return;
	}

	data_dispatcher_feed_gyro(0.0F);

}
