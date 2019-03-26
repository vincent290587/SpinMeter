/*
 * lis2dw12_wrapper.h
 *
 *  Created on: 25 mars 2019
 *      Author: Vincent
 */

#ifndef SOURCE_SENSORS_LIS2DW12_WRAPPER_H_
#define SOURCE_SENSORS_LIS2DW12_WRAPPER_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


void lis2dw12_meas_trigger(void);

void lis2dw12_wrapper_init(void);

bool lis2dw12_wrapper_is_updated(void);

void lis2dw12_wrapper_sensor_refresh(void);


#ifdef __cplusplus
}
#endif

#endif /* SOURCE_SENSORS_LIS2DW12_WRAPPER_H_ */
