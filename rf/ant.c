

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "ant.h"
#include "nordic_common.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "nrf_soc.h"
#include "nrf_delay.h"
#include "nrf_sdh.h"
#include "app_timer.h"

#ifdef ANT_STACK_SUPPORT_REQD
#include "ant_key_manager.h"
#include "ant_search_config.h"
#include "ant_bsc.h"
#include "ant_interface.h"

#include "Model.h"
#include "segger_wrapper.h"

#include "bsc.h"


/**< Application's ANT observer priority. You shouldn't need to modify this value. */
#define APP_ANT_OBSERVER_PRIO       1


/**
 * Event handler for background search
 */
static void ant_evt_bs (ant_evt_t * p_ant_evt)
{
	ret_code_t err_code = NRF_SUCCESS;

	switch (p_ant_evt->event)
	{
	case EVENT_RX:
	{
		uint16_t m_last_device_id;
		uint8_t m_last_rssi = 0;

        m_last_rssi = p_ant_evt->message.ANT_MESSAGE_aucExtData[5];
        m_last_device_id = uint16_decode(p_ant_evt->message.ANT_MESSAGE_aucExtData);

        if (m_last_device_id)
        {
        	m_last_device_id = uint16_decode(p_ant_evt->message.ANT_MESSAGE_aucExtData);

    		LOG_WARNING("Dev. ID 0x%04X %d", m_last_device_id, (int8_t)m_last_rssi);

        	ant_device_manager_search_add(m_last_device_id, m_last_rssi);
        }

	} break;
	case EVENT_RX_FAIL:
		break;
	case EVENT_RX_FAIL_GO_TO_SEARCH:
		break;
	case EVENT_RX_SEARCH_TIMEOUT:
		break;
	case EVENT_CHANNEL_CLOSED:
		break;
	}

	APP_ERROR_CHECK(err_code);
}


/**@brief Function for dispatching a ANT stack event to all modules with a ANT stack event handler.
 *
 * @details This function is called from the ANT Stack event interrupt handler after a ANT stack
 *          event has been received.
 *
 * @param[in] p_ant_evt  ANT stack event.
 */
void ant_evt_handler(ant_evt_t * p_ant_evt, void * p_context)
{
	W_SYSVIEW_RecordEnterISR();

	switch(p_ant_evt->channel) {

	case BSC_CHANNEL_NUMBER:
//		TODO BSC event ant_evt_bsc (p_ant_evt);
		break;

	default:
		break;
	}

    W_SYSVIEW_RecordExitISR();
}

NRF_SDH_ANT_OBSERVER(m_ant_observer, APP_ANT_OBSERVER_PRIO, ant_evt_handler, 0);

/**@brief Function for initializing the timer module.
 */
void ant_timers_init(void)
{
	bsc_init();
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ant_stack_init(void)
{
	ret_code_t err_code;

	err_code = nrf_sdh_ant_enable();
	APP_ERROR_CHECK(err_code);

	err_code = ant_plus_key_set(ANTPLUS_NETWORK_NUMBER);
	APP_ERROR_CHECK(err_code);

}

/**@brief Function for HRM profile initialization.
 *
 * @details Initializes the HRM profile and open ANT channel.
 */
static void ant_profile_setup(void)
{

	// CAD
	bsc_profile_setup();

}

/**
 *
 */
void ant_setup_init(void) {

	ant_profile_setup();

}

/**
 *
 */
void ant_setup_start(uint16_t bsc_id)
{
	ret_code_t err_code;

	// TODO Set the new device ID.
//	err_code = sd_ant_channel_id_set(BSC_CHANNEL_NUMBER,
//			bsc_id,
//			BSC_DEVICE_TYPE,
//			WILDCARD_TRANSMISSION_TYPE);
//	APP_ERROR_CHECK(err_code);

	// Open the ANT channels
	bsc_profile_start();

	LOG_INFO("ANT started");
}

/**
 *
 */
void ant_tasks(void) {

}


#endif
