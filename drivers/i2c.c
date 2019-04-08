/*
 * I2C.c
 *
 *  Created on: 26 f�vr. 2017
 *      Author: Vincent
 */

#include "boards.h"
#include "parameters.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "millis.h"
#include "nrf_pwr_mgmt.h"
#include "Model.h"
#include "segger_wrapper.h"

#include "i2c.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     1

#define MAX_PENDING_TRANSACTIONS    40

NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);



/**
 *
 */
void i2c_init(void) {

	ret_code_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = SCL_PIN_NUMBER,
       .sda                = SDA_PIN_NUMBER,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = true,
	   .hold_bus_uninit    = true
    };

    err_code = nrf_twi_mngr_init(&m_nrf_twi_mngr, &config);
    APP_ERROR_CHECK(err_code);

}

/**
 *
 */
void i2c_uninit(void) {

	nrf_twi_mngr_uninit(&m_nrf_twi_mngr);

}

void i2c_scan(void) {

}

/**
 *
 * @param p_transaction
 */
void i2c_schedule(nrf_twi_mngr_transaction_t const * p_transaction) {

	/* Start master transfer */
	ret_code_t ret_val = nrf_twi_mngr_schedule(&m_nrf_twi_mngr, p_transaction);
	APP_ERROR_CHECK(ret_val);
}


/**
 *
 * @param p_transaction
 */
uint32_t i2c_perform(nrf_drv_twi_config_t const *    p_config,
        nrf_twi_mngr_transfer_t const * p_transfers,
        uint8_t                         number_of_transfers,
        void                            (* user_function)(void)) {

	/* Start master transfer */
	ret_code_t ret_val = nrf_twi_mngr_perform(&m_nrf_twi_mngr,
			p_config,
			p_transfers,
			number_of_transfers,
			user_function);

	APP_ERROR_CHECK(ret_val);
	return ret_val;
}


