/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "board_lib.h"
#include "startup_task.h"
#include "gimbal_control_task.h"
#include "movement_control_task.h"
#include "referee_processing_task.h"
#include "control_input_task.h"
#include "launcher_control_task.h"
#include "imu_processing_task.h"
#include "robot_config.h"
#include "buzzing_task.h"
#include "motor_config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//FreeRTOS definitions
#define ISR_SEMAPHORE_COUNT 1
#define QUEUE_SIZE 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

//Input control task definitions
TaskHandle_t gimbal_control_task_handle;
TaskHandle_t movement_control_task_handle;
TaskHandle_t referee_processing_task_handle;
TaskHandle_t control_input_task_handle;
TaskHandle_t launcher_control_task_handle;
TaskHandle_t buzzing_task_handle;
TaskHandle_t motor_calib_task_handle;
TaskHandle_t usb_task_handle;
TaskHandle_t imu_processing_task_handle;
TaskHandle_t telemetry_task_handle;
TaskHandle_t hud_task_handle;

EventGroupHandle_t gimbal_event_group;
EventGroupHandle_t chassis_event_group;
EventGroupHandle_t launcher_event_group;

SemaphoreHandle_t usb_continue_semaphore;

QueueHandle_t gyro_data_queue;
QueueHandle_t accel_data_queue;
QueueHandle_t mag_data_queue;

QueueHandle_t telem_data_queue;
QueueHandle_t buzzing_task_msg;
QueueHandle_t xvr_data_queue;
QueueHandle_t uart_data_queue;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void) {

}

__weak unsigned long getRunTimeCounterValue(void) {
	return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask,
		signed char *pcTaskName) {
	/* Run time stack overflow checking is performed if
	 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
	 called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
	/* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
		StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize) {
	*ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
	*ppxTimerTaskStackBuffer = &xTimerStack[0];
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
	/* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	gimbal_event_group = xEventGroupCreate();
	chassis_event_group = xEventGroupCreate();
	launcher_event_group = xEventGroupCreate();

	usb_continue_semaphore = xSemaphoreCreateBinary();

	//Controls when different task can execute

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	gyro_data_queue = xQueueCreate(5, sizeof(gyro_data_t));
	accel_data_queue = xQueueCreate(5, sizeof(accel_data_t));
	mag_data_queue = xQueueCreate(5, sizeof(mag_data_t));
	//telem_data_queue = xQueueCreate(10, sizeof(telem_data_struct_t));
	buzzing_task_msg = xQueueCreate(48, sizeof(uint8_t));
	uart_data_queue = xQueueCreate(5, sizeof(ref_msg_t));

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	//todo: adjust priorities
	//Threads creation
	xTaskCreate(imu_processing_task, "IMU_task",
	1024, (void*) 1, (UBaseType_t) 15,
			&imu_processing_task_handle);

	xTaskCreate(motor_calib_task, "motor_calib_task",
	configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 9,
			&motor_calib_task_handle);

	if (gimbal_event_group == NULL) {
		//error handler implement next time!
	} else {
		xTaskCreate(gimbal_control_task, "gimbal_task",
		configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 7,
				&gimbal_control_task_handle);

	}

	if (chassis_event_group == NULL) {
		//error handler
	} else {
		xTaskCreate(movement_control_task, "chassis_task",
		configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 4,
				&movement_control_task_handle);
	}

	if (launcher_event_group == NULL) {
		//error handler
	} else {
		xTaskCreate(launcher_control_task, "launcher_task",
		configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 4,
				&launcher_control_task_handle);
	}

	xTaskCreate(control_input_task, "RC_task",
	1024, (void*) 1, (UBaseType_t) 4,
			&control_input_task_handle);
	xTaskCreate(referee_processing_task, "referee_task", 2048, (void*) 1,
			(UBaseType_t) 2, &referee_processing_task_handle);
	xTaskCreate(buzzing_task, "buzzer_task",
	configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 1, &buzzing_task_handle);


  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
