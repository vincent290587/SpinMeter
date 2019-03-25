/*
 * bsc.c
 *
 *  Created on: 12 mrt. 2019
 *      Author: v.golle
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "Model.h"
#include "segger_wrapper.h"

#include "ant.h"
#include "nordic_common.h"
#include "app_error.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "app_timer.h"
#include "ant_device_manager.h"

#ifdef ANT_STACK_SUPPORT_REQD
#include "ant_key_manager.h"
#include "ant_search_config.h"
#include "ant_bsc.h"
#include "ant_interface.h"

#include "Model.h"
#include "segger_wrapper.h"

#include "bsc.h"

#define ANT_DELAY                       APP_TIMER_TICKS(30000)


/** @snippet [ANT BSC RX Instance] */
#define WHEEL_CIRCUMFERENCE         2070                                                            /**< Bike wheel circumference [mm] */
#define BSC_EVT_TIME_FACTOR         1024                                                            /**< Time unit factor for BSC events */
#define BSC_RPM_TIME_FACTOR         60                                                              /**< Time unit factor for RPM unit */
#define BSC_MS_TO_KPH_NUM           36                                                              /**< Numerator of [m/s] to [kph] ratio */
#define BSC_MS_TO_KPH_DEN           10                                                              /**< Denominator of [m/s] to [kph] ratio */
#define BSC_MM_TO_M_FACTOR          1000                                                            /**< Unit factor [m/s] to [mm/s] */
#define BSC_SPEED_UNIT_FACTOR       (BSC_MS_TO_KPH_DEN * BSC_MM_TO_M_FACTOR)                        /**< Speed unit factor */
//#define SPEED_COEFFICIENT           (WHEEL_CIRCUMFERENCE * BSC_EVT_TIME_FACTOR * BSC_MS_TO_KPH_NUM) /**< Coefficient for speed value calculation */
#define CADENCE_COEFFICIENT         (BSC_EVT_TIME_FACTOR * BSC_RPM_TIME_FACTOR)                     /**< Coefficient for cadence value calculation */
#define SPEED_COEFFICIENT           (WHEEL_CIRCUMFERENCE * BSC_EVT_TIME_FACTOR * BSC_MS_TO_KPH_NUM \
		/ BSC_MS_TO_KPH_DEN)                      /**< Coefficient for speed value calculation */

static void ant_bsc_evt_handler(ant_bsc_profile_t * p_profile, ant_bsc_evt_t event);


BSC_SENS_CHANNEL_CONFIG_DEF(m_ant_bsc,
                            BSC_CHANNEL_NUM,
                            CHAN_ID_TRANS_TYPE,
                            SENSOR_TYPE,
                            CHAN_ID_DEV_NUM,
                            ANTPLUS_NETWORK_NUM);
BSC_SENS_PROFILE_CONFIG_DEF(m_ant_bsc,
                            true,
                            true,
                            ANT_BSC_PAGE_5,
                            ant_bsc_evt_handler);


static ant_bsc_simulator_t  m_ant_bsc;    /**< Simulator used to simulate profile data. */

/**
 *
 */
static void ant_bsc_evt_handler(ant_bsc_profile_t * p_profile, ant_bsc_evt_t event)
{
	switch (event)
	{
	case ANT_BSC_PAGE_0_UPDATED:
		/* fall through */
	case ANT_BSC_PAGE_1_UPDATED:
		/* fall through */
	case ANT_BSC_PAGE_2_UPDATED:
		/* fall through */
	case ANT_BSC_PAGE_3_UPDATED:
		/* fall through */
	case ANT_BSC_PAGE_4_UPDATED:
		/* fall through */
	case ANT_BSC_PAGE_5_UPDATED:
		/* Log computed value */
		break;

	case ANT_BSC_COMB_PAGE_0_UPDATED:
	{
		ant_bsc_simulator_one_iteration(&m_ant_bsc_simulator);
	} break;

	default:
		break;
	}
}

/**
 *
 */
void ant_evt_bsc (ant_evt_t * p_ant_evt)
{
	ret_code_t err_code = NRF_SUCCESS;

	uint16_t pusDeviceNumber = 0;
	uint8_t pucDeviceType    = 0;
	uint8_t pucTransmitType  = 0;

	switch (p_ant_evt->event)
	{
	case EVENT_RX:
		if (!is_cad_init) {
			sd_ant_channel_id_get (BSC_CHANNEL_NUMBER,
					&pusDeviceNumber, &pucDeviceType, &pucTransmitType);

			if (pusDeviceNumber) {
				is_cad_init = 1;

				memset(&m_speed_calc_data, 0, sizeof(m_speed_calc_data));
				memset(&m_cadence_calc_data, 0, sizeof(m_cadence_calc_data));
			}
		}
		ant_bsc_disp_evt_handler(p_ant_evt, &m_ant_bsc);
		break;
	case EVENT_RX_FAIL:
		break;
	case EVENT_RX_FAIL_GO_TO_SEARCH:
		memset(&m_speed_calc_data, 0, sizeof(m_speed_calc_data));
		memset(&m_cadence_calc_data, 0, sizeof(m_cadence_calc_data));
		break;
	case EVENT_RX_SEARCH_TIMEOUT:
		break;
	case EVENT_CHANNEL_CLOSED:
		is_cad_init = 0;
		err_code = app_timer_start(m_sec_bsc, ANT_DELAY, NULL);
		APP_ERROR_CHECK(err_code);
		break;
	}

}

/**@brief Function for initializing the timer module.
 */
void bsc_init(void)
{
	ret_code_t err_code;

	err_code = app_timer_create(&m_sec_bsc, APP_TIMER_MODE_SINGLE_SHOT, bsc_connect);
	APP_ERROR_CHECK(err_code);

}

/**
 *
 */
void bsc_profile_setup(void) {

	ret_code_t err_code;

    err_code = ant_bsc_sens_init(&m_ant_bsc,
                                 BSC_SENS_CHANNEL_CONFIG(m_ant_bsc),
                                 BSC_SENS_PROFILE_CONFIG(m_ant_bsc));
    APP_ERROR_CHECK(err_code);

    m_ant_bsc.BSC_PROFILE_manuf_id     = BSC_MF_ID;
    m_ant_bsc.BSC_PROFILE_serial_num   = BSC_SERIAL_NUMBER;
    m_ant_bsc.BSC_PROFILE_hw_version   = BSC_HW_VERSION;
    m_ant_bsc.BSC_PROFILE_sw_version   = BSC_SW_VERSION;
    m_ant_bsc.BSC_PROFILE_model_num    = BSC_MODEL_NUMBER;

    err_code = ant_bsc_sens_open(&m_ant_bsc);
    APP_ERROR_CHECK(err_code);

    err_code = ant_state_indicator_channel_opened();
    APP_ERROR_CHECK(err_code);

	ant_search_config.channel_number = BSC_CHANNEL_NUMBER;
	err_code = ant_search_init(&ant_search_config);
	APP_ERROR_CHECK(err_code);

    /** @snippet [ANT BSC simulator init] */
    const ant_bsc_simulator_cfg_t simulator_cfg =
    {
        .p_profile      = &m_ant_bsc,
        .device_type    = SENSOR_TYPE,
    };
    /** @snippet [ANT BSC simulator init] */

    ant_bsc_simulator_init(&m_ant_bsc_simulator, &simulator_cfg, true);
}

/**
 *
 */
void bsc_profile_start(void) {

	ret_code_t err_code;

	err_code = ant_bsc_disp_open(&m_ant_bsc);
	APP_ERROR_CHECK(err_code);
}


#endif
