/*
 * gpio.h
 *
 *  Created on: 19 sept. 2018
 *      Author: Vincent
 */

#ifndef DRIVERS_GPIO_H_
#define DRIVERS_GPIO_H_

#include <stdint.h>
#include "nordic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

void gpio_set(uint32_t gpio_nb_);

void gpio_clear(uint32_t gpio_nb_);

void gpio_toggle(uint32_t gpio_nb_);

bool gpio_get(uint32_t gpio_nb_);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_GPIO_H_ */
