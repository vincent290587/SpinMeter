/*
 * unit_testing.cpp
 *
 *  Created on: 3 nov. 2018
 *      Author: Vincent
 */

#include <cmath>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "Model_tdd.h"
#include "fram.h"
#include "utils.h"

#include "order1_filter.h"

#define TEST_FILTRE_NB    15

#define TEST_ROLLOF_NB    48759

bool test_fram(void) {

	LOG_INFO("Testing FRAM...");

	fram_init_sensor();

	if (!u_settings.isConfigValid())
		return false;

	if (!u_settings.resetConfig())
		return false;

	if (!u_settings.isConfigValid())
		return false;

	LOG_INFO("FRAM OK");

	return true;
}

bool test_power_zone(void) {

	PowerZone p_zones;
	uint32_t timestamp = 0;

	LOG_INFO("Testing suffer score...");

	for (int i=0; i < 116; i++) p_zones.addPowerData(110, (timestamp++)*1000);
	for (int i=0; i < 158; i++) p_zones.addPowerData(175, (timestamp++)*1000);
	for (int i=0; i < 1831; i++) p_zones.addPowerData(206, (timestamp++)*1000);
	for (int i=0; i < 812; i++) p_zones.addPowerData(242, (timestamp++)*1000);
	for (int i=0; i < 330; i++) p_zones.addPowerData(401, (timestamp++)*1000);

	LOG_INFO("Time spent in PZ %u", p_zones.getTimeTotal());

	if (p_zones.getTimeTotal() < 4) return false;

	if (fabsf(p_zones.getTimeMax() - 1831) > 2) return false;

	return true;
}

