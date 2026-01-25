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
#include "stdio.h"
#include "hwt605.h"
#include "imu_kalman.h"
#include "ReadData.h "
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DOG_SWITCH AUTO_OFFROAD

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
Dog dog;
uint8_t Rx_Temp=0;
QueueHandle_t leg1_pidHandle;
extern wit_t hwt_angle;
static imu_kalman_t g_imu_kf;
unsigned char buf[64]={0};
CAN_RxHeaderTypeDef RxHeader;
osThreadId AutoModeTaskHandle;
osThreadId CalculateTaskHandle;
osThreadId RemoteContrlTaskHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void AutoMode(void const * argument);
void calculateFunc(void const * argument);
void RC_Ctrl(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

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

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityAboveNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */  
  osThreadDef(CalculateTask, calculateFunc, osPriorityNormal, 0, 512);
  CalculateTaskHandle = osThreadCreate(osThread(CalculateTask), NULL);
  
  #if DOG_SWITCH==RC_MODE
  dog.dog_mode=RC_MODE;
  osThreadDef(RemoteContrlTask, RC_Ctrl, osPriorityNormal, 0, 256);
  RemoteContrlTaskHandle = osThreadCreate(osThread(RemoteContrlTask), NULL);
  #elif DOG_SWITCH==AUTO_OFFROAD
  dog.dog_mode=AUTO_OFFROAD;  
  osThreadDef(AutoModeTask, AutoMode, osPriorityNormal, 0, 512);
  AutoModeTaskHandle = osThreadCreate(osThread(AutoModeTask), NULL);
  #elif DOG_SWITCH==AUTO_TASK
  dog.dog_mode=AUTO_TASK;
  osThreadDef(AutoModeTask, AutoMode, osPriorityNormal, 0, 512);
  AutoModeTaskHandle = osThreadCreate(osThread(AutoModeTask), NULL);
  #endif
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
	hwt605_Init();
//	HAL_UART_Receive_IT(&huart2, (uint8_t *)&Rx_Temp, 1);
  /* Infinite loop */
  for(;;)
  {
//      printf("1.38,1.23\n");
//    HAL_UART_Transmit(&huart2,&d,sizeof(d),100);
//    VCP_ReadTask();
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
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
  Dog_ParaInit(&dog);
  /* Infinite loop */
  for(;;)
  {
    dogTaskCtrl(&dog);
    osDelay(1);
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
    osDelay(1);
  }
  /* USER CODE END RC_Ctrl */
}
void AutoMode(void const * argument){
  imu_kalman_init(&g_imu_kf, 0.03f, 0.1f);
  uint32_t now;
  uint32_t last_tick = osKernelSysTick();
  for(;;){
    now = osKernelSysTick();
    float dt = (now - last_tick) * 0.001f; // ms -> s
    last_tick = now;
    if (dt > 0.0f)
    {
      imu_kalman_step_from_hwt(&g_imu_kf, &hwt_angle, dt);
      dog.location.yaw = imu_kalman_get_angle_deg(&g_imu_kf, 2);
      last_tick = osKernelSysTick();
      }
      printf("%f,%f\n",hwt_angle.fYaw,dog.location.yaw);
      osDelay(10);
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{ 
		SBUS_Reveive(Rx_Temp);
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&Rx_Temp, 1);
	}
}
/* USER CODE END Application */
