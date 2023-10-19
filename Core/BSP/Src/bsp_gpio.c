/*
 * bsp_gpio.c
 *
 *  Created on: Sep 7, 2021
 *      Author: wx
 */
#include "board_lib.h"
#include "bsp_gpio.h"

void laser_on()
{ //set to reset for open day
	HAL_GPIO_WritePin(LASER_GPIO_GPIO_Port, LASER_GPIO_Pin, GPIO_PIN_RESET);
}

void laser_off()
{
	HAL_GPIO_WritePin(LASER_GPIO_GPIO_Port, LASER_GPIO_Pin, GPIO_PIN_RESET);
}
