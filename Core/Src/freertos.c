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
#include "usart.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include "stdio.h"
#include "hwt605.h"
#include "imu_kalman.h"
#include "ReadData.h "
#include "vofa_Debug.h"
#include "A28_RC.h"
#include "world.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DOG_SWITCH RC_MODE

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
WorldModel world;
uint8_t Rx_Temp[10] = {0};
static imu_kalman_t g_imu_kf;
unsigned char buf[64] = {0};
extern wit_t hwt_angle;
extern uint8_t lora_rx_byte;
extern vcp_message_t msg;
CAN_RxHeaderTypeDef RxHeader;
osThreadId AutoModeTaskHandle;
osThreadId CalculateTaskHandle;
osThreadId RemoteContrlTaskHandle;
osThreadId TargetSwitchTaskHandle;
osThreadId PlanningTaskHandle;
static UBaseType_t g_defaultTaskMinHwmWords = (UBaseType_t)0xFFFFFFFFu;
static uint32_t g_defaultTaskSampleCnt = 0;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void AutoMode(void const *argument);
void calculateFunc(void const *argument);
void RC_Ctrl(void const *argument);
void TargetSwitch(void const *argument);
void Planning(void const *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
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
void MX_FREERTOS_Init(void)
{
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
  osThreadDef(CalculateTask, calculateFunc, osPriorityNormal, 0, 1024);
  CalculateTaskHandle = osThreadCreate(osThread(CalculateTask), NULL);
#if DOG_SWITCH == RC_MODE
  world.dog.dog_mode = RC_MODE;
  osThreadDef(RemoteContrlTask, RC_Ctrl, osPriorityNormal, 0, 256);
  RemoteContrlTaskHandle = osThreadCreate(osThread(RemoteContrlTask), NULL);
#elif DOG_SWITCH == AUTO_OFFROAD
  world.dog.dog_mode = AUTO_OFFROAD;
  osThreadDef(AutoModeTask, AutoMode, osPriorityNormal, 0, 512);
  AutoModeTaskHandle = osThreadCreate(osThread(AutoModeTask), NULL);
#endif
  //  osThreadDef(TargetSwitchTask, TargetSwitch, osPriorityNormal, 0, 512);
  //  TargetSwitchTaskHandle = osThreadCreate(osThread(TargetSwitchTask), NULL);
  //  osThreadDef(PlanningTask, Planning, osPriorityNormal, 0, 512);
  //  PlanningTaskHandle = osThreadCreate(osThread(PlanningTask), NULL);

  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  hwt605_Init(); // IMU初始化
  world_init(&world);
  HAL_UART_Receive_IT(&huart3, &lora_rx_byte, 1);
  /* Infinite loop */
  for (;;)
  {
    UBaseType_t hwm_words = uxTaskGetStackHighWaterMark(NULL);
    if (hwm_words < g_defaultTaskMinHwmWords)
    {
      g_defaultTaskMinHwmWords = hwm_words;
    }
    g_defaultTaskSampleCnt++;
    //       HAL_UART_Transmit(&huart2,&d,sizeof(d),100);
    printf("%.2f,%.2f\n", msg.xdata, msg.ydata);
    if ((g_defaultTaskSampleCnt % 10u) == 0u)
    {
      printf("defaultTask min hwm: %lu words (%lu bytes)\n",
             (unsigned long)g_defaultTaskMinHwmWords,
             (unsigned long)(g_defaultTaskMinHwmWords * sizeof(StackType_t)));
    }
    VCP_ReadTask();
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
void calculateFunc(void const *argument)
{
  /* USER CODE BEGIN calculateFunc */
  osDelay(2000);
  Dog_ParaInit(&world.dog);
  /* Infinite loop */
  for (;;)
  {
    dogTaskCtrl(&world.dog);
    osDelay(5);
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
void RC_Ctrl(void const *argument)
{
  /* USER CODE BEGIN RC_Ctrl */

  /* Infinite loop */
  for (;;)
  {
    A28_RC(&world.dog);
    osDelay(100);
  }
  /* USER CODE END RC_Ctrl */
}

void AutoMode(void const *argument)
{
  imu_kalman_init(&g_imu_kf, 0.03f, 0.1f);
  uint32_t now;
  uint32_t last_tick = osKernelSysTick();
  for (;;)
  {
    now = osKernelSysTick();
    float dt = (now - last_tick) * 0.001f; // ms -> s
    last_tick = now;
    if (dt > 0.0f)
    {
      imu_kalman_step_from_hwt(&g_imu_kf, &hwt_angle, dt);
      world.dog.location.yaw = imu_kalman_get_angle_deg(&g_imu_kf, 2);
      last_tick = osKernelSysTick();
    }
    printf("%f,%f\n", hwt_angle.fYaw, world.dog.location.yaw);
    osDelay(100);
  }
}

void TargetSwitch(void const *argument)
{
  for (;;)
  {
    task_switch(&world.dog, &world.box_groups, &world.return_field);
    osDelay(1000);
  }
}
void Planning(void const *argument)
{
  for (;;)
  {
    dog_Planning(&world.dog);
    osDelay(100);
  }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   if (huart == &huart2)
//   {
//     vofa_GetData(&Rx_Temp[0]);
// //      SBUS_Reveive(Rx_Temp[0]);
//     HAL_UART_Receive_IT(&huart2, &Rx_Temp[0], 6);
//   }
// }

// void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
// {
//   if (huart == &huart2)
//   {
//     __HAL_UART_CLEAR_OREFLAG(huart);
//     __HAL_UART_CLEAR_FEFLAG(huart);
//     __HAL_UART_CLEAR_NEFLAG(huart);
//     HAL_UART_Receive_IT(&huart2, &Rx_Temp[0], 6);
//   }
// }
/* USER CODE END Application */
