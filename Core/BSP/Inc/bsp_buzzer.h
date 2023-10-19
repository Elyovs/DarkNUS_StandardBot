/*
 * bsp_buzzer.h
 *
 *  Created on: May 23, 2021
 *      Author: wx
 */

#ifndef BSP_INC_BSP_BUZZER_H_
#define BSP_INC_BSP_BUZZER_H_


#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"

void buzzer(uint16_t freq);
void buzzer_init();

#endif /* BSP_INC_BSP_BUZZER_H_ */
