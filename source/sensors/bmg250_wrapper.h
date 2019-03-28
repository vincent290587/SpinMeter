/*
 * bmg250_wrapper.h
 *
 *  Created on: 25 mars 2019
 *      Author: Vincent
 */

#ifndef SOURCE_SENSORS_BMG250_WRAPPER_H_
#define SOURCE_SENSORS_BMG250_WRAPPER_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


void bmg250_wrapper_init(void);

void bmg250_wrapper_schedule_sensor(void);

bool bmg250_wrapper_is_updated(void);

void bmg250_wrapper_clear_updated(void);

void bmg250_wrapper_sensor_refresh(void);


#ifdef __cplusplus
}
#endif

#endif /* SOURCE_SENSORS_BMG250_WRAPPER_H_ */
