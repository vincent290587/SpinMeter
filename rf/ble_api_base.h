/*
 * ble_api.h
 *
 *  Created on: 21 d�c. 2018
 *      Author: Vincent
 */

#ifndef RF_BLE_API_BASE_H_
#define RF_BLE_API_BASE_H_

#include "g_structs.h"

#ifdef __cplusplus
extern "C" {
#endif


void ble_init(void);

void ble_uninit(void);

void ble_nus_tasks(void);

void ble_nus_log_cadence(uint32_t cadence, uint32_t d_cad);

void ble_nus_log_text(const char * text);


#ifdef __cplusplus
}
#endif

#endif /* RF_BLE_API_BASE_H_ */
