/*
 * bsp_buzzer.c
 *
 *  Created on: May 23, 2021
 *      Author: wx
 */

#include "board_lib.h"
#include "bsp_buzzer.h"

void buzzer(uint16_t freq)
{
	if (freq == 0)
	{
		htim4.Instance->CCR3 = 0;
	}
	else
	{
		__HAL_TIM_SET_PRESCALER(&htim4, (84 * 500/freq));
		htim4.Instance->CCR3 = 500;
	}
}


void buzzer_init()
{
	  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	  htim4.Instance->CCR3 = 0;
}


