/*
 * gpio.c

 *
 *  Created on: 19 sept. 2018
 *      Author: Vincent
 */

#include <stdlib.h>
#include "gpio.h"
#include "Model_tdd.h"
#include "segger_wrapper.h"

extern int sockfd;

void gpio_set(uint16_t gpio_nb_) {

	if (gpio_nb_ == 0) {
		exit(-5);
	}
}

uint8_t gpio_get(uint16_t gpio_nb_) {


	return 0;
}

void register_btn_press(uint8_t btn_index) {


}

void btn_task(void) {


}
