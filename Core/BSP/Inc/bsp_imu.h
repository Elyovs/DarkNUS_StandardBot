/*
 * bsp_imu.h
 *
 *  Created on: Sep 7, 2021
 *      Author: wx
 */

#ifndef BSP_INC_BSP_IMU_H_
#define BSP_INC_BSP_IMU_H_



float inv_sqrt(float x);
void gyro_write_byte(uint8_t const reg, uint8_t const data);
uint8_t gyro_read_byte(uint8_t const reg);
uint8_t gyro_read_bytes(uint8_t const reg, uint8_t* pData, uint8_t len);
uint8_t gyro_read_bytes_DMA(uint8_t const regAddr, uint8_t* pData, uint8_t len);
void accel_write_byte(uint8_t const reg, uint8_t const data);
uint8_t accel_read_byte(uint8_t const reg);
uint8_t accel_read_bytes(uint8_t reg, uint8_t pData[], uint8_t len);
uint8_t accel_read_bytes_DMA(uint8_t const reg, uint8_t* pData, uint8_t len);
uint8_t gyro_init();
uint8_t accel_init();
float get_bmi_temp();
uint8_t mag_read_single_reg(uint8_t reg);
void mag_write_single_reg(uint8_t reg, uint8_t data);
void mag_read_multi_reg(uint8_t reg, uint16_t len, uint8_t *data);
void reset_imu_data();
void gyro_offset_cali();
void imu_init();
void accel_get_data();
void accel_process_data();
void gyro_get_data();
void gyro_process_data();
uint8_t ist8310_init();
void ist8310_get_data();
void imu_start_ints();
HAL_StatusTypeDef gyro_txrx_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                              uint16_t Size);

HAL_StatusTypeDef accel_txrx_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                              uint16_t Size);
#endif /* BSP_INC_BSP_IMU_H_ */
