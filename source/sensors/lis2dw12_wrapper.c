/*
 * lis2dw12_wrapper.c
 *
 *  Created on: 25 mars 2019
 *      Author: Vincent
 */


#include <stdint.h>
#include "i2c.h"
#include "boards.h"
#include "helper.h"
#include "lis2dw12_reg.h"
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

static lis2dw12_ctx_t dev_ctx;
static lis2dw12_reg_t int_route;
static bool m_is_updated;
static float acceleration_mg[3];

static int32_t _lis_i2c_read(void *handle, uint8_t reg_addr, uint8_t *p_data, uint16_t length) {

	nrf_twi_mngr_transfer_t const xfer[] =
	{
			I2C_READ_REG(LIS2DW12_I2C_ADD_L, &reg_addr, p_data, length)
	};

	i2c_perform(NULL, xfer, sizeof(xfer) / sizeof(xfer[0]), NULL);

	return 0;
}

static int32_t _lis_i2c_write(void *handle, uint8_t reg_addr, uint8_t *data, uint16_t length) {

	uint8_t p_data[30];

	ASSERT(length < 30 - 1);

	p_data[0] = reg_addr;
	for (int i=0; i< length; i++) {
		p_data[i+1] = data[i];
	}

	nrf_twi_mngr_transfer_t const xfer[] =
	{
			I2C_WRITE(LIS2DW12_I2C_ADD_L, p_data, length+1)
	};

	i2c_perform(NULL, xfer, sizeof(xfer) / sizeof(xfer[0]), NULL);

	return 0;
}

static void _lis2dw12_readout_cb(ret_code_t result, void * p_user_data) {

	axis3bit16_t *data_raw_acceleration = (axis3bit16_t*)p_user_data;

	ASSERT(p_user_data);

	if (result) {
		LOG_WARNING("LIS2DW read error");
		return;
	}

	acceleration_mg[0] = LIS2DW12_FROM_FS_8g_LP1_TO_mg(data_raw_acceleration->i16bit[0]);
	acceleration_mg[1] = LIS2DW12_FROM_FS_8g_LP1_TO_mg(data_raw_acceleration->i16bit[1]);
	acceleration_mg[2] = LIS2DW12_FROM_FS_8g_LP1_TO_mg(data_raw_acceleration->i16bit[2]);

	m_is_updated = true;

	LOG_DEBUG("LIS2DW read");

}

static void _lis2dw12_read_sensor(void) {

	static axis3bit16_t data_raw_acceleration;

	static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND readout_reg[] = {LIS2DW12_OUT_X_L};

	static nrf_twi_mngr_transfer_t const lis12_readout_transfer[] =
	{
			I2C_READ_REG(LIS2DW12_I2C_ADD_L, readout_reg, data_raw_acceleration.u8bit , 6)
	};

	static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
	{
			.callback            = _lis2dw12_readout_cb,
			.p_user_data         = data_raw_acceleration.u8bit,
			.p_transfers         = lis12_readout_transfer,
			.number_of_transfers = sizeof(lis12_readout_transfer) / sizeof(lis12_readout_transfer[0])
	};

	i2c_schedule(&transaction);

}

static void _int1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	// schedule sensor reading
	_lis2dw12_read_sensor();
}

void lis2dw12_wrapper_init(void)
{
	uint8_t whoamI, rst;

	dev_ctx.write_reg = _lis_i2c_write;
	dev_ctx.read_reg = _lis_i2c_read;
	dev_ctx.handle = NULL;

	/*
	 * Check device ID
	 */
	lis2dw12_device_id_get(&dev_ctx, &whoamI);
	if (whoamI != LIS2DW12_ID)
		LOG_ERROR("LIS2DW not found");

	/*
	 * Restore default configuration
	 */
	lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);
	do {
		lis2dw12_reset_get(&dev_ctx, &rst);
	} while (rst);

	/*
	 *  Enable Block Data Update
	 */
	lis2dw12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

	/*
	 * Set full scale
	 */
	lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_8g);

	/*
	 * Configure filtering chain
	 *
	 * Accelerometer - filter path / bandwidth
	 */
	lis2dw12_filter_path_set(&dev_ctx, LIS2DW12_LPF_ON_OUT);
	lis2dw12_filter_bandwidth_set(&dev_ctx, LIS2DW12_ODR_DIV_4);

	/*
	 * Configure power mode
	 */
	//lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_HIGH_PERFORMANCE);
	lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit);

	/*
	 * Set Output Data Rate
	 */
	lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_25Hz);

	/*
	 * Enable interrupt generation on Wake-Up INT1 pin
	 *
	 */
	lis2dw12_pin_int1_route_get(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
	int_route.ctrl4_int1_pad_ctrl.int1_wu = PROPERTY_ENABLE;
	lis2dw12_pin_int1_route_set(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);

	nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	in_config.pull = NRF_GPIO_PIN_PULLDOWN;

	ret_code_t err_code = nrfx_gpiote_in_init(LIS_INT1, &in_config, _int1_handler);
	APP_ERROR_CHECK(err_code);

	nrfx_gpiote_in_event_enable(LIS_INT1, true);

}

void lis2dw12_wrapper_set_wake(void)
{
	uint8_t whoamI, rst;

	dev_ctx.write_reg = _lis_i2c_write;
	dev_ctx.read_reg = _lis_i2c_read;
	dev_ctx.handle = NULL;

	/*
	 * Check device ID
	 */
	lis2dw12_device_id_get(&dev_ctx, &whoamI);
	if (whoamI != LIS2DW12_ID)
		LOG_ERROR("LIS2DW not found");

	/*
	 * Restore default configuration
	 */
	lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);
	do {
		lis2dw12_reset_get(&dev_ctx, &rst);
	} while (rst);

	/*
	 * Set full scale
	 */
	lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_2g);

	/*
	 * Configure power mode
	 */
	lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit);

	/*
	 * Set Output Data Rate
	 */
	lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_1Hz6_LP_ONLY);

	/*
	 * Apply high-pass digital filter on Wake-Up function
	 */
	lis2dw12_filter_path_set(&dev_ctx, LIS2DW12_HIGH_PASS_ON_OUT);

	/*
	 * Apply high-pass digital filter on Wake-Up function
	 * Duration time is set to zero so Wake-Up interrupt signal
	 * is generated for each X,Y,Z filtered data exceeding the
	 * configured threshold
	 */
	lis2dw12_wkup_dur_set(&dev_ctx, 0);

	/*
	 * Set wake-up threshold
	 *
	 * Set Wake-Up threshold: 1 LSb corresponds to FS_XL/2^6
	 */
	lis2dw12_wkup_threshold_set(&dev_ctx, 2);

	/*
	 * Enable interrupt generation on Wake-Up INT2 pin
	 *
	 */
	lis2dw12_pin_int2_route_get(&dev_ctx, &int_route.ctrl5_int2_pad_ctrl);
	int_route.ctrl5_int2_pad_ctrl.int2_drdy = PROPERTY_ENABLE;
	lis2dw12_pin_int2_route_set(&dev_ctx, &int_route.ctrl5_int2_pad_ctrl);

	nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	in_config.pull = NRF_GPIO_PIN_PULLDOWN;

	ret_code_t err_code = nrfx_gpiote_in_init(LIS_INT2, &in_config, _int1_handler);
	APP_ERROR_CHECK(err_code);

	nrfx_gpiote_in_event_enable(LIS_INT2, true);

}

