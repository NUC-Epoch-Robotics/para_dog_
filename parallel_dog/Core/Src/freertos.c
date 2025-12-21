/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "queue.h"
#include "motor_control.h"
#include "postrue_control.h"
#include "math.h"
#include "rc.h"
#include "bsp_sbus.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include "hwt605.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
Dog dog;
uint8_t Rx_Temp=0;
QueueHandle_t leg1_pidHandle;
extern wit_t angle;
	unsigned char buf[10]={0};

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId CalculateTaskHandle;
osThreadId RemoteContrlTasHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void calculateFunc(void const * argument);
void RC_Ctrl(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	leg1_pidHandle=xQueueCreate(20,2*5*sizeof(float));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityAboveNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of CalculateTask */
  osThreadDef(CalculateTask, calculateFunc, osPriorityNormal, 0, 512);
  CalculateTaskHandle = osThreadCreate(osThread(CalculateTask), NULL);

  /* definition and creation of RemoteContrlTas */
  osThreadDef(RemoteContrlTas, RC_Ctrl, osPriorityNormal, 0, 256);
  RemoteContrlTasHandle = osThreadCreate(osThread(RemoteContrlTas), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
	memset(dog.leg,0,sizeof(Leg) * 4);
	hwt605_Init();
	Dog_ParaInit(&dog);
	HAL_UART_Receive_IT(&huart2,(uint8_t *)&Rx_Temp,1);

//	osDelay(5000);
//	leg[0].state=STEP_FORE; 
  /* Infinite loop */
  for(;;)
  {
//	dog.state=STAND_UP_;
//	osDelay(4000);
//	dog.state=WALK_FORWARD;
//		osDelay(4000);
//	dog.state=TURN_RIGHT;
	VCP_Read(buf,64);
	CDC_Transmit_FS(buf,sizeof(buf));
	osDelay(1000);
//	dog.state=TURN_LEFT;
//		osDelay(4000);
//	dog.state=LOWWALK_FORWARD;
//		osDelay(4000);
	//	dog.state=JUMP_FORWARD;
//		osDelay(4000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_calculateFunc */
/**
* @brief Function implementing the CalculateTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_calculateFunc */
void calculateFunc(void const * argument)
{
  /* USER CODE BEGIN calculateFunc */	
	osDelay(1000);
  /* Infinite loop */
  for(;;)
  {

	dogTaskCtrl(&dog);
	osDelay(10);
  }
  /* USER CODE END calculateFunc */
}

/* USER CODE BEGIN Header_RC_Ctrl */
/**
* @brief Function implementing the RemoteContrlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RC_Ctrl */
void RC_Ctrl(void const * argument)
{
  /* USER CODE BEGIN RC_Ctrl */
  /* Infinite loop */
  for(;;)
  {
	rc_remote_ctrl(&dog);
    osDelay(10);
  }
  /* USER CODE END RC_Ctrl */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart==&huart2){ 
		SBUS_Reveive(Rx_Temp);
		HAL_UART_Receive_IT(&huart2,(uint8_t *)&Rx_Temp,1);
	}
}
/* USER CODE END Application */
