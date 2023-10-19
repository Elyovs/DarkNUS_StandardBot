/*
 * bsp_led.c
 *
 *  Created on: Jan 19, 2020
 *      Author: Kai Yang
 */

#include "bsp_led.h"
#include "tim.h"


/* calls HAL library function to directly write pin.             */
/* LEDs are active low, so SET (1) == off, while RESET (0) == on */
/* Definitions for LED port and pins are given in main.h file    */
void led_green_off(void){
	htim5.Instance->CCR2 = 0;
}

void led_green_on(void){
	htim5.Instance->CCR2 = 255;

}

void led_green_toggle(void){
	if (htim5.Instance->CCR2 > 0)
	{
		htim5.Instance->CCR2 = 0;
	}
	else
	{
		htim5.Instance->CCR2 = 255;
	}
}

void led_red_off(void){
	htim5.Instance->CCR3 = 0;
}

void led_red_on(void){
	htim5.Instance->CCR3 = 255;
}

void led_red_toggle(void){
	if (htim5.Instance->CCR3 > 0)
	{
		htim5.Instance->CCR3 = 0;
	}
	else
	{
		htim5.Instance->CCR3 = 255;
	}
}

void led_off(void){
	htim5.Instance->CCR3 = 0;
	htim5.Instance->CCR2 = 0;
	htim5.Instance->CCR1 = 0;
}

void led_on(void){
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	htim5.Instance->CCR3 = 255;
	htim5.Instance->CCR2 = 255;
	htim5.Instance->CCR1 = 255;
}
void led_toggle(void){
}

void status_led(uint16_t led_no, uint8_t led_state)
{
	assert_param(IS_GPIO_ALL_PERIPH(GPIOG));
	if (led_state == on_led)
	{
	  /* points to address of GPIOx register and does bitwise XOR with bit GPIO_Pin */
		switch(led_no)
		{
		case 1:
			htim5.Instance->CCR3 = 255;//(0x94);
			htim5.Instance->CCR2 = 0;//(0xe1);
			htim5.Instance->CCR1 = 0;///(0xa0);
			break;
		case 2:
			htim5.Instance->CCR3 = 0;//(0x25);
			htim5.Instance->CCR2 = 255;//(0x25);
			htim5.Instance->CCR1 = 0;//(0x64);
			break;
		case 3:
			htim5.Instance->CCR3 = 0;//(0x52);
			htim5.Instance->CCR2 = 0;//(0x0c);
			htim5.Instance->CCR1 = 255;//(0xcf);
			break;
		case 4:
			htim5.Instance->CCR3 = 255;//(0x6e);
			htim5.Instance->CCR2 = 0;//(0x7a);
			htim5.Instance->CCR1 = 255;//(0xf4);
			break;
		case 5:
			htim5.Instance->CCR3 = 0;
			htim5.Instance->CCR2 = 0;
			htim5.Instance->CCR1 = 0;
			break;
		case 6:
			htim5.Instance->CCR3 = 0;
			htim5.Instance->CCR2 = 0;
			htim5.Instance->CCR1 = 0;
			break;
		case 7:
			htim5.Instance->CCR3 = 0;
			htim5.Instance->CCR2 = 0;
			htim5.Instance->CCR1 = 0;
			break;
		case 8:
			htim5.Instance->CCR3 = 0;
			htim5.Instance->CCR2 = 0;
			htim5.Instance->CCR1 = 0;
			break;
		case 9:
			htim5.Instance->CCR3 = 0;
			htim5.Instance->CCR2 = 0;
			htim5.Instance->CCR1 = 0;
			break;

		default:
			break;
		}
	}

}
