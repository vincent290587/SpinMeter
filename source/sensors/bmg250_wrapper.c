/*
 * bmg250_wrapper.c
 *
 *  Created on: 25 mars 2019
 *      Author: Vincent
 */


#include <stdint.h>
#include "i2c.h"
#include "helper.h"
#include "bmg250.h"
#include "nrf_twi_mngr.h"
#include "segger_wrapper.h"


#define I2C_READ_REG(addr, p_reg_addr, p_buffer, byte_cnt) \
		NRF_TWI_MNGR_WRITE(addr, p_reg_addr, 1, NRF_TWI_MNGR_NO_STOP), \
		NRF_TWI_MNGR_READ (addr, p_buffer, byte_cnt, 0)

#define I2C_READ_REG_REP_STOP(addr, p_reg_addr, p_buffer, byte_cnt) \
		NRF_TWI_MNGR_WRITE(addr, p_reg_addr, 1, 0), \
		NRF_TWI_MNGR_READ (addr, p_buffer, byte_cnt, 0)

#define I2C_WRITE(addr, p_data, byte_cnt) \
		NRF_TWI_MNGR_WRITE(addr, p_data, byte_cnt, 0)

static struct bmg250_dev m_gyro;
static struct bmg250_sensor_data m_gyro_data;

static bool m_is_updated = false;

static void _bmg_delay_ms(uint32_t period) {
	nrf_delay_ms(period);
}

static int8_t _bmg_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *p_data, uint16_t length) {

	nrf_twi_mngr_transfer_t const xfer[] =
	{
			I2C_READ_REG(dev_id, &reg_addr, p_data, length)
	};

	i2c_perform(NULL, xfer, sizeof(xfer) / sizeof(xfer[0]), NULL);

	return BMG250_OK;
}

static int8_t _bmg_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t length) {

	uint8_t p_data[30];

	ASSERT(length < 30 - 1);

	p_data[0] = reg_addr;
	for (int i=0; i< length; i++) {
		p_data[i+1] = data[i];
	}

	nrf_twi_mngr_transfer_t const xfer[] =
	{
			I2C_WRITE(dev_id, p_data, length+1)
	};

	i2c_perform(NULL, xfer, sizeof(xfer) / sizeof(xfer[0]), NULL);

	return BMG250_OK;
}

static void _bmg250_readout_cb(ret_code_t result, void * p_user_data) {

	W_SYSVIEW_RecordEnterISR();

	uint8_t idx = 0;
	uint8_t lsb;
	uint8_t msb;
	uint8_t *data_array = (uint8_t*)p_user_data;

	ASSERT(p_user_data);

	if (result) {
		LOG_WARNING("BMG250 read error");
		return;
	}

	lsb = data_array[idx++];
	msb = data_array[idx++];
	/* gyro X axis data */
	m_gyro_data.x = (int16_t)(((uint16_t)msb << 8) | lsb);

	lsb = data_array[idx++];
	msb = data_array[idx++];
	/* gyro Y axis data */
	m_gyro_data.y = (int16_t)(((uint16_t)msb << 8) | lsb);

	lsb = data_array[idx++];
	msb = data_array[idx++];
	/* gyro Z axis data */
	m_gyro_data.z = (int16_t)(((uint16_t)msb << 8) | lsb);

	/* update sensor-time data as 0 */
	m_gyro_data.sensortime = 0;

	m_is_updated = true;

	LOG_DEBUG("BMG250 read (%d)", m_gyro_data.z);

	W_SYSVIEW_RecordExitISR();

}

void bmg250_wrapper_schedule_sensor(void) {

	static uint8_t p_ans_buffer[12] = {0};

	static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND readout_reg[] = {BMG250_DATA_ADDR};

	static nrf_twi_mngr_transfer_t const bmg250_readout_transfer[] =
	{
			I2C_READ_REG(BMG250_I2C_ADDR, readout_reg, p_ans_buffer , 6)
	};

	static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
	{
			.callback            = _bmg250_readout_cb,
			.p_user_data         = p_ans_buffer,
			.p_transfers         = bmg250_readout_transfer,
			.number_of_transfers = sizeof(bmg250_readout_transfer) / sizeof(bmg250_readout_transfer[0])
	};

	i2c_schedule(&transaction);

}

void bmg250_wrapper_init(void) {

	int8_t rslt = BMG250_OK;

	/* Sensor interface over I2C */
	m_gyro.dev_id = BMG250_I2C_ADDR;
	m_gyro.interface = BMG250_I2C_INTF;
	m_gyro.read = _bmg_i2c_read;
	m_gyro.write = _bmg_i2c_write;
	m_gyro.delay_fct = _bmg_delay_ms;
	m_gyro.power_mode = BMG250_GYRO_NORMAL_MODE;

	rslt = bmg250_init(&m_gyro);
	APP_ERROR_CHECK(rslt);

	rslt = bmg250_set_power_mode(&m_gyro);

	struct bmg250_cfg gyro_cfg;

	/* Read the set configuration from the sensor */
	rslt = bmg250_get_sensor_settings(&gyro_cfg, &m_gyro);

	if (rslt == BMG250_OK) {
		/* Selecting the ODR as 100Hz */
		gyro_cfg.odr = BMG250_ODR_25HZ;
		/* Selecting the bw as Normal mode */
		gyro_cfg.bw = BMG250_BW_OSR2_MODE;
		/* Selecting the range as 2000 Degrees/second */
		gyro_cfg.range = BMG250_RANGE_2000_DPS;
	} else {
		LOG_ERROR("Could not get sensor settings");
	}

	rslt = bmg250_set_sensor_settings(&gyro_cfg, &m_gyro);

	struct bmg250_int_settg int_conf;

	/* Enable the desired settings to be set in the sensor
	 * Refer below for all possible settings */
	int_conf.int_channel = BMG250_INT_CHANNEL_1;
	int_conf.int_type = BMG250_DATA_RDY_INT;
	int_conf.int_pin_settg.output_en = BMG250_ENABLE;
	int_conf.int_pin_settg.output_mode = BMG250_PUSH_PULL;
	int_conf.int_pin_settg.output_type = BMG250_ACTIVE_HIGH;
	int_conf.int_pin_settg.edge_ctrl = BMG250_EDGE_TRIGGER;
	int_conf.int_pin_settg.input_en = BMG250_DISABLE;

	/* Set the desired configuration in the sensor */
	rslt = bmg250_set_int_config(&int_conf, &m_gyro);

	LOG_WARNING("BMG250 Init success (%u)", rslt);
}

bool bmg250_wrapper_is_updated(void) {
	return m_is_updated;
}

void bmg250_wrapper_sensor_refresh(void) {

	if (!m_is_updated) return;
	m_is_updated = false;

}
