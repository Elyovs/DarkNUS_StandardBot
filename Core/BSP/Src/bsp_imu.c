/*
 * bsp_imu.c
 *
 *  Created on: Sep 7, 2021
 *      Author: wx
 */

#include "board_lib.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_spi.h"
#include "spi.h"
#include "i2c.h"
#include "typedefs.h"
#include "BMI088reg.h"
#include "ist8310_reg.h"
#include "bsp_imu.h"
#include "imu_processing_task.h"

#define IMU_HSPI			hspi1
#define IST_I2C				hi2c3
#define BMI_ACCEL_NSS_LOW	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define BMI_ACCEL_NSS_HIGH	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define BMI_GYRO_NSS_LOW	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define BMI_GYRO_NSS_HIGH	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define ACCEL_MAX_RANGE		32768
#define ACCEL_MAX_GS		6
#define GYRO_MAX_RANGE 		32768
#define GYRO_MAX_W			1000


static uint8_t accel_buff[6];
static uint8_t gyro_buff[6];
static uint8_t temp_buff[3];
static uint8_t ist_read_buffer[6] = { 0 };
static imu_raw_t imu_data;
static uint8_t imu_init_status = 0;

/**
 * @brief  fast inverse square-root, to calculate 1/Sqrt(x)
 * @param  x: the number need to be calculated
 * @retval 1/Sqrt(x)
 * @usage  call in imu_ahrs_update() function
 */
float inv_sqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*) &y;

	i = 0x5f3759df - (i >> 1);
	y = *(float*) &i;
	y = y * (1.5f - (halfx * y * y));

	return y;
}

void gyro_write_byte(uint8_t const reg, uint8_t const data) {
	BMI_GYRO_NSS_LOW;
	uint8_t tx, rx;
	tx = reg;
	HAL_SPI_TransmitReceive(&IMU_HSPI, &tx, &rx, 1, 100);
	tx = data;
	HAL_SPI_TransmitReceive(&IMU_HSPI, &tx, &rx, 1, 100);
	BMI_GYRO_NSS_HIGH;
	return;
}

uint8_t gyro_read_byte(uint8_t const reg) {
	BMI_GYRO_NSS_LOW;
	uint8_t rx, tx;
	tx = reg | 0x80;
	HAL_SPI_TransmitReceive(&IMU_HSPI, &tx, &rx, 1, 100);
	HAL_SPI_Receive(&IMU_HSPI, &rx, 1, 100);
	BMI_GYRO_NSS_HIGH;
	return rx;
}

uint8_t gyro_read_bytes(uint8_t const reg, uint8_t *pData, uint8_t len) {
	uint8_t tx, rx;
	BMI_GYRO_NSS_LOW;
	tx = reg | 0x80;
	HAL_SPI_TransmitReceive(&IMU_HSPI, &tx, &rx, 1, 100);
	HAL_SPI_Receive(&IMU_HSPI, pData, len, 100);
	BMI_GYRO_NSS_HIGH;
	return 0;
}

uint8_t gyro_read_bytes_DMA(uint8_t const regAddr, uint8_t *pData, uint8_t len) {
	uint8_t tx, rx;
	BMI_GYRO_NSS_LOW;
	tx = regAddr | 0x80;
	HAL_SPI_TransmitReceive(&IMU_HSPI, &tx, &rx, 1, 100);
	uint8_t rx_status = HAL_SPI_TransmitReceive_DMA(&IMU_HSPI, pData, pData, len);
	if (rx_status != HAL_OK) {
		BMI_GYRO_NSS_HIGH;
		return 1;
	}
	return 0;
}

void accel_write_byte(uint8_t const reg, uint8_t const data) {
	uint8_t tx, rx;
	BMI_ACCEL_NSS_LOW;
	tx = reg;
	HAL_SPI_TransmitReceive(&IMU_HSPI, &tx, &rx, 1, 100);
	tx = data;
	HAL_SPI_TransmitReceive(&IMU_HSPI, &tx, &rx, 1, 100);
	BMI_ACCEL_NSS_HIGH;
	return;
}

uint8_t accel_read_byte(uint8_t const reg) {
	uint8_t tx, rx;
	BMI_ACCEL_NSS_LOW;
	tx = reg | 0x80;
	HAL_SPI_TransmitReceive(&IMU_HSPI, &tx, &rx, 1, 100);
	tx = 0x55;
	HAL_SPI_Receive(&IMU_HSPI, &rx, 1, 100);
	HAL_SPI_Receive(&IMU_HSPI, &rx, 1, 100);
	BMI_ACCEL_NSS_HIGH;
	return rx;
}

uint8_t accel_read_bytes(uint8_t reg, uint8_t pData[], uint8_t len) {
	BMI_ACCEL_NSS_LOW;
	uint8_t tx, rx;
	tx = reg | 0x80;
	HAL_SPI_TransmitReceive(&IMU_HSPI, &tx, &rx, 1, 100);
	HAL_SPI_Receive(&IMU_HSPI, &rx, 1, 100);
	HAL_SPI_Receive(&IMU_HSPI, pData, len, 100);
	BMI_ACCEL_NSS_HIGH;
	return 0;
}

uint8_t accel_read_bytes_DMA(uint8_t const reg, uint8_t *pData, uint8_t len) {
	BMI_ACCEL_NSS_LOW;
	uint8_t tx, rx;
	tx = reg | 0x80;
	HAL_SPI_TransmitReceive(&IMU_HSPI, &tx, &rx, 1, 100);
	HAL_SPI_Receive(&IMU_HSPI, &rx, 1, 100);
	uint8_t rx_status = HAL_SPI_TransmitReceive_DMA(&IMU_HSPI, pData, pData, len);
	if (rx_status != HAL_OK) {
		BMI_ACCEL_NSS_HIGH;
		return 1;
	}
	return 0;
}

uint8_t gyro_init() {
	gyro_write_byte(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
	HAL_Delay(10);
	gyro_write_byte(BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE);
	HAL_Delay(2);
	int16_t gyro_id = gyro_read_byte(BMI088_GYRO_CHIP_ID);
	if (gyro_id != BMI088_GYRO_CHIP_ID_VALUE) {
		NVIC_SystemReset();
		return 1;
	}
	HAL_Delay(2);
	gyro_write_byte(BMI088_GYRO_RANGE, BMI088_GYRO_500);
	HAL_Delay(2);
	gyro_write_byte(BMI088_GYRO_BANDWIDTH,
			(BMI088_GYRO_BANDWIDTH_MUST_Set | BMI088_GYRO_1000_116_HZ));
	HAL_Delay(2);
	gyro_write_byte(BMI088_GYRO_INT3_INT4_IO_CONF,
			(BMI088_GYRO_INT3_GPIO_HIGH | BMI088_GYRO_INT3_GPIO_PP));
	HAL_Delay(2);
	gyro_write_byte(BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3);
	HAL_Delay(2);

	return 0;
	//blah
}

uint8_t accel_init() {
	uint8_t accel_id = accel_read_byte(BMI088_ACC_CHIP_ID);
	HAL_Delay(2);
	accel_id = accel_read_byte(BMI088_ACC_CHIP_ID);
	HAL_Delay(2);
	accel_write_byte(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
	HAL_Delay(10);

	//Attempt to read...twice because once doesn't cause it to initiate
	accel_id = accel_read_byte(BMI088_ACC_CHIP_ID);
	HAL_Delay(2);
	accel_id = accel_read_byte(BMI088_ACC_CHIP_ID);
	HAL_Delay(2);
	if (accel_id != BMI088_ACC_CHIP_ID_VALUE) {
		//add in an error code for this someday
		return 1;
	}

	accel_write_byte(BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON);
	HAL_Delay(2);
	accel_write_byte(BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE);
	HAL_Delay(2);

	//set accelerometer to normal from suspend
	accel_write_byte(BMI088_ACC_CONF,
			(BMI088_ACC_NORMAL | BMI088_ACC_100_HZ | BMI088_ACC_CONF_MUST_Set));
	HAL_Delay(2);

	accel_write_byte(BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G);
	HAL_Delay(2);

	accel_write_byte(BMI088_INT1_IO_CTRL,
			(BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW));
	HAL_Delay(2);
	return 0;
}

//NOT BEING USED CURRENTLY
//todo: add in IMU heater pid thing
float get_bmi_temp() {
	//??? which NSS lol
	int16_t temp_int11;
	uint16_t temp_uint11;
	accel_read_bytes(BMI088_TEMP_L, temp_buff, 3);
	temp_uint11 = (temp_buff[1] * 8) + (temp_buff[2] / 32);
	if (temp_uint11 > 1023) {
		temp_int11 = temp_uint11 - 2048;
	} else {
		temp_int11 = temp_uint11;
	}
	float temperature = temp_int11 * 0.125 / temp_buff[1] + 23;
	return temperature;
}

uint8_t mag_read_single_reg(uint8_t reg) {
	uint8_t data = 0;
	HAL_I2C_Mem_Read(&hi2c3, IST8310_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 5);
	return data;
}

void mag_write_single_reg(uint8_t reg, uint8_t data) {
	HAL_I2C_Mem_Write(&hi2c3, IST8310_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 5);
}

void mag_read_multi_reg(uint8_t reg, uint16_t len, uint8_t *data) {
	HAL_I2C_Mem_Read(&hi2c3, IST8310_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, data, len, 10);
}

void mag_read_multi_reg_dma(uint8_t reg, uint16_t len, uint8_t *data) {
	HAL_I2C_Mem_Read_DMA(&hi2c3, IST8310_ADDRESS << 1, reg,
	I2C_MEMADD_SIZE_8BIT, data, len);
}

void reset_imu_data() {

}

//find the offset of the gyro
void gyro_offset_cali() {
	int32_t gyro_x_total = 0;
	int32_t gyro_y_total = 0;
	int32_t gyro_z_total = 0;
	for (uint16_t i = 0; i < 100; i++) {
		gyro_read_bytes(BMI088_GYRO_X_L, gyro_buff, 6);
		gyro_x_total += (int16_t)(gyro_buff[1] << 8 | gyro_buff[0]);
		gyro_y_total += (int16_t)(gyro_buff[3] << 8 | gyro_buff[2]);
		gyro_z_total += (int16_t)(gyro_buff[5] << 8 | gyro_buff[4]);
		HAL_Delay(2);
	}
	imu_data.gx_offset = gyro_x_total / 100;
	imu_data.gy_offset = gyro_y_total / 100;
	imu_data.gz_offset = gyro_z_total / 100;

}

void imu_init() {
	//trigger NSS low and high for both to initialise both gyro and accel into SPI mode
	BMI_ACCEL_NSS_LOW;
	BMI_GYRO_NSS_LOW;
	HAL_Delay(100);
	BMI_ACCEL_NSS_HIGH;
	BMI_GYRO_NSS_HIGH;
	gyro_init();
	accel_init();
	ist8310_init();
//	gyro_offset_cali();

}

void imu_start_ints() {
	gyro_write_byte(BMI088_GYRO_CTRL, BMI088_DRDY_ON);
	vTaskDelay(10);
	accel_write_byte(BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT);
	vTaskDelay(10);
	mag_write_single_reg(0x0B, 0x08); //enable drdy pin, pull to low on drdy
	imu_init_status = 1;
}

void accel_get_data() {
	accel_read_bytes(BMI088_ACCEL_XOUT_L, accel_buff, 6);
//	if (rx_status == HAL_OK) {
//		spi_rdy = 0;
//		accel_rx = 1;
//	}
	//imu_data.temp = get_bmi_temp();
}

void accel_process_data() {
	int16_t accel_data;
	accel_data = (accel_buff[1] << 8 | accel_buff[0]);

	//convert raw data into m/s^2
	imu_data.accel_data.ax = (float)accel_data * BMI088_ACCEL_6G_SEN;
	accel_data = (accel_buff[3] << 8 | accel_buff[2]);

	//convert raw data into m/s^2
	imu_data.accel_data.ay = (float)accel_data * BMI088_ACCEL_6G_SEN;
	accel_data = (accel_buff[5] << 8 | accel_buff[4]);

	//convert raw data into m/s^2
	imu_data.accel_data.az = (float)accel_data * BMI088_ACCEL_6G_SEN;
	imu_data.accel_data.last_accel_update = HAL_GetTick();

	//send data into the imu processing task
	accel_data_ready(imu_data.accel_data);
}

void gyro_get_data() {
	gyro_read_bytes(BMI088_GYRO_X_L, gyro_buff, 6);
//	if (rx_status == HAL_OK) {
//		spi_rdy = 0;
//		gyro_rx = 1;
//	}
}

void gyro_process_data() {
	int16_t gyro_data;
	gyro_data = (gyro_buff[1] << 8 | gyro_buff[0]);
	gyro_data -= imu_data.gx_offset;
	//convert raw data into radians/s
	imu_data.gyro_data.gx = (float)gyro_data * BMI088_GYRO_500_SEN;
	gyro_data = (gyro_buff[3] << 8 | gyro_buff[2]);
	gyro_data -= imu_data.gy_offset;

	//convert raw data into radians/s
	imu_data.gyro_data.gy = (float)gyro_data * BMI088_GYRO_500_SEN;
	gyro_data = (gyro_buff[5] << 8 | gyro_buff[4]);
	gyro_data -= imu_data.gz_offset;

	//convert raw data into radians/s
	imu_data.gyro_data.gz = (float)gyro_data * BMI088_GYRO_500_SEN;
	imu_data.gyro_data.last_gyro_update = HAL_GetTick();
	//send data into the imu processing task
	gyro_data_ready(imu_data.gyro_data);
}

uint8_t ist8310_init() {
	HAL_GPIO_WritePin(IST_RST_GPIO_Port, IST_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(IST_RST_GPIO_Port, IST_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(2);

	uint8_t ist_id = mag_read_single_reg(IST8310_WHO_AM_I);
	if (ist_id != IST8310_DEVICE_ID_A) {
		//reset mag values
		imu_data.mag_data.mx = 0;
		imu_data.mag_data.my = 0;
		imu_data.mag_data.mz = 0;
		return 0;
	}
	HAL_Delay(2);
	mag_write_single_reg(0x41, 0x09); //average over 2 data
	HAL_Delay(2);
	mag_write_single_reg(0x42, 0xC0); //set to C0
	HAL_Delay(2);
	mag_write_single_reg(0x0A, 0x0B); //100Hz
	//mag_en_flag = 1;

	return 0;
}

void ist8310_get_data() {
	//fun
	mag_read_multi_reg_dma(IST8310_R_XL, 5, ist_read_buffer);
	imu_data.mag_data.last_mag_update = HAL_GetTick();

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (imu_init_status == 1) {
		if (hspi1.State == HAL_SPI_STATE_READY) {
			if (GPIO_Pin == GYRO_INT_Pin) {
				gyro_get_data();
				gyro_process_data();
				//ist8310_get_data();
			}
			if (GPIO_Pin == ACC_INT_Pin) {
				accel_get_data();
				accel_process_data();
			}
		}
		if (GPIO_Pin == IST_INT_Pin) {
			if (hi2c3.State == HAL_I2C_STATE_READY) {
				ist8310_get_data();
			}
		}
	}
}

/*
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (gyro_rx) {
		BMI_GYRO_NSS_HIGH;
		gyro_process_data();
		gyro_rx = 0;
	} else if (accel_rx) {
		BMI_ACCEL_NSS_HIGH;
		accel_process_data();
		accel_rx = 0;
	}
	spi_rdy = 1;
	spi_timeout = HAL_GetTick();
}
*/

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	int16_t mag_buffer[3];
	mag_buffer[0] = ist_read_buffer[1] << 8 | ist_read_buffer[0];
	mag_buffer[1] = ist_read_buffer[3] << 8 | ist_read_buffer[2];
	mag_buffer[2] = ist_read_buffer[5] << 8 | ist_read_buffer[4];

	imu_data.mag_data.mx = mag_buffer[0];
	imu_data.mag_data.my = mag_buffer[1];
	imu_data.mag_data.mz = mag_buffer[2];
	mag_data_ready(imu_data.mag_data);
}
