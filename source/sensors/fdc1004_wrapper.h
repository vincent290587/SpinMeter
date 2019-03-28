/*
 * fdc1004_wrapper.h
 *
 *  Created on: 28 mrt. 2019
 *      Author: v.golle
 */

#ifndef SOURCE_SENSORS_FDC1004_WRAPPER_H_
#define SOURCE_SENSORS_FDC1004_WRAPPER_H_


#ifdef	__cplusplus
extern "C" {
#endif


void fdc1004_meas_trigger(void);

void fdc1004_wrapper_init(void);

void fdc1004_wrapper_sensor_tasks(void);


#ifdef	__cplusplus
}
#endif

#endif /* SOURCE_SENSORS_FDC1004_WRAPPER_H_ */
