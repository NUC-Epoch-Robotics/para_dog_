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
#include "app_remote_rx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DOG_SWITCH RC_MODE
#define Signal_DogInitOK (1U << 0)
#define Signal_WorldInitOK (1U << 1)
#define STACK_WORDS_DEFAULT_TASK 256U
#define STACK_WORDS_CALCULATE_TASK 1024U
#define STACK_WORDS_REMOTE_TASK 512U
#define STACK_WORDS_PLANNING_TASK 512U
#define STACK_WORDS_SENSOR_TASK 512U
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
extern uint8_t lora_rx_byte[10];
extern vcp_message_t msg;
extern uint8_t vofa_rx_frame[6];
extern uint8_t process_buf[RX_BUF_SIZE];
extern uint16_t process_len;
CAN_RxHeaderTypeDef RxHeader;
osThreadId SensorTaskHandle;
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
void Sensor(void const *argument);
void calculateFunc(void const *argument);
void RC_Ctrl(void const *argument);
void TargetSwitch(void const *argument);
void Planning(void const *argument);
static void PrintTaskStackUsage(void);
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityAboveNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(CalculateTask, calculateFunc, osPriorityNormal, 0, 1024);
  CalculateTaskHandle = osThreadCreate(osThread(CalculateTask), NULL);
#if DOG_SWITCH == RC_MODE
  world.dog.dog_mode = RC_MODE;
  osThreadDef(RemoteContrlTask, RC_Ctrl, osPriorityNormal, 0, 512);
  RemoteContrlTaskHandle = osThreadCreate(osThread(RemoteContrlTask), NULL);
#elif DOG_SWITCH == AUTO_MODE
  world.dog.dog_mode = AUTO_MODE;
  // osThreadDef(TargetSwitchTask, TargetSwitch, osPriorityNormal, 0, 512);
  // TargetSwitchTaskHandle = osThreadCreate(osThread(TargetSwitchTask), NULL);
  osThreadDef(PlanningTask, Planning, osPriorityNormal, 0, 512);
  PlanningTaskHandle = osThreadCreate(osThread(PlanningTask), NULL);
#endif
  osThreadDef(SensorTask, Sensor, osPriorityNormal, 0, 512);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

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
  world_init(&world);
  osSignalSet(CalculateTaskHandle, Signal_WorldInitOK);
  vTaskSuspend(NULL);
  /* Infinite loop */
  for (;;)
  {
    // printf("%.2f\n",hwt_angle.fYaw);
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
  osSignalWait(Signal_WorldInitOK, osWaitForever);
#if DOG_SWITCH == AUTO_MODE
  osSignalWait(VCP_SIGNAL, osWaitForever); // 等待VCP初始化完成
  standUP_FPC(&world.dog, 216.5f);
  if (PlanningTaskHandle != NULL)
  {
    osSignalSet(PlanningTaskHandle, Signal_DogInitOK);
  }
  if (TargetSwitchTaskHandle != NULL)
  {
    osSignalSet(TargetSwitchTaskHandle, Signal_DogInitOK);
  }
#elif (DOG_SWITCH == RC_MODE)
  standUP_FPC(&world.dog, 216.5f);
#endif
  /* Infinite loop */
  for (;;)
  {
    dogTaskCtrl(&world.dog);
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
void RC_Ctrl(void const *argument)
{
  /* USER CODE BEGIN RC_Ctrl */

  vofa_init(&huart2);
  HAL_UART_Receive_IT(&huart2, vofa_rx_frame, 6);
  // uint8_t local_buf[RX_BUF_SIZE];
  // uint16_t local_len = 0;

  // app_remote_rx_init(&huart3);
  // app_remote_rx_start();
  // app_remote_thread(RemoteContrlTaskHandle);
  /* Infinite loop */
  for (;;)
  {
    vofa_GetData(vofa_rx_frame);
    vofa_rc(&world.dog);
    // for (uint8_t leg = 0; leg < 4; leg++)
    // {
    //   printf("%.2f,%.2f,%.2f\n", world.dog.leg[leg].motor_ctrl_linkf.cmd.T, world.dog.leg[leg].motor_ctrl_linkb.cmd.T, world.dog.leg[leg].motor_ctrl_linkf.cmd.Pos);
    // }
    osDelay(100);
  }

  /* USER CODE END RC_Ctrl */
}

void Sensor(void const *argument)
{

  hwt605_Init(); // IMU初始化
#if DOG_SWITCH == AUTO_MODE
  uint8_t drain_rounds;
  uint32_t now_tick;
  uint32_t last_stack_log_tick;
  VCP_ReadTask_Init(osThreadGetId());
  osEvent evt = osSignalWait(VCP_SIGNAL, osWaitForever);
  if (evt.status == osEventSignal && (evt.value.signals & VCP_SIGNAL))
  {
    osSignalSet(CalculateTaskHandle, VCP_SIGNAL);
  }
  last_stack_log_tick = osKernelSysTick();
#endif

  for (;;)
  {
#if DOG_SWITCH == AUTO_MODE
    evt = osSignalWait(VCP_SIGNAL, osWaitForever);
    if (evt.status == osEventSignal && (evt.value.signals & VCP_SIGNAL))
    {
      drain_rounds = 0U;
      do
      {
        VCP_ReadTask();
        drain_rounds++;
      } while ((VCP_GetRxCount() > 0U) && (drain_rounds < 10U));
      world.dog.location.pos.x = msg.xdata;
      world.dog.location.pos.y = msg.ydata;
      now_tick = osKernelSysTick();
      if ((now_tick - last_stack_log_tick) >= 2000U)
      {
        PrintTaskStackUsage();
        last_stack_log_tick = now_tick;
      }
    }
#endif
    printf("%.2f,%.2f,%.2f\n", world.dog.location.roll, world.dog.location.pitch, world.dog.location.yaw);
    osDelay(100);
  }
}

static void PrintTaskStackUsage(void)
{
  UBaseType_t free_words;
  uint32_t free_bytes;
  uint32_t cfg_bytes;
  uint32_t used_bytes;

  if (defaultTaskHandle != NULL)
  {
    free_words = uxTaskGetStackHighWaterMark((TaskHandle_t)defaultTaskHandle);
    free_bytes = (uint32_t)free_words * sizeof(StackType_t);
    cfg_bytes = STACK_WORDS_DEFAULT_TASK * sizeof(StackType_t);
    used_bytes = (cfg_bytes >= free_bytes) ? (cfg_bytes - free_bytes) : 0U;
    printf("[STACK] default used=%luB free_min=%luB cfg=%luB\r\n", used_bytes, free_bytes, cfg_bytes);
  }

  if (CalculateTaskHandle != NULL)
  {
    free_words = uxTaskGetStackHighWaterMark((TaskHandle_t)CalculateTaskHandle);
    free_bytes = (uint32_t)free_words * sizeof(StackType_t);
    cfg_bytes = STACK_WORDS_CALCULATE_TASK * sizeof(StackType_t);
    used_bytes = (cfg_bytes >= free_bytes) ? (cfg_bytes - free_bytes) : 0U;
    printf("[STACK] calculate used=%luB free_min=%luB cfg=%luB\r\n", used_bytes, free_bytes, cfg_bytes);
  }

  if (SensorTaskHandle != NULL)
  {
    free_words = uxTaskGetStackHighWaterMark((TaskHandle_t)SensorTaskHandle);
    free_bytes = (uint32_t)free_words * sizeof(StackType_t);
    cfg_bytes = STACK_WORDS_SENSOR_TASK * sizeof(StackType_t);
    used_bytes = (cfg_bytes >= free_bytes) ? (cfg_bytes - free_bytes) : 0U;
    printf("[STACK] sensor used=%luB free_min=%luB cfg=%luB\r\n", used_bytes, free_bytes, cfg_bytes);
  }

#if DOG_SWITCH == RC_MODE
  if (RemoteContrlTaskHandle != NULL)
  {
    free_words = uxTaskGetStackHighWaterMark((TaskHandle_t)RemoteContrlTaskHandle);
    free_bytes = (uint32_t)free_words * sizeof(StackType_t);
    cfg_bytes = STACK_WORDS_REMOTE_TASK * sizeof(StackType_t);
    used_bytes = (cfg_bytes >= free_bytes) ? (cfg_bytes - free_bytes) : 0U;
    printf("[STACK] remote used=%luB free_min=%luB cfg=%luB\r\n", used_bytes, free_bytes, cfg_bytes);
  }
#elif DOG_SWITCH == AUTO_MODE
  if (PlanningTaskHandle != NULL)
  {
    free_words = uxTaskGetStackHighWaterMark((TaskHandle_t)PlanningTaskHandle);
    free_bytes = (uint32_t)free_words * sizeof(StackType_t);
    cfg_bytes = STACK_WORDS_PLANNING_TASK * sizeof(StackType_t);
    used_bytes = (cfg_bytes >= free_bytes) ? (cfg_bytes - free_bytes) : 0U;
    printf("[STACK] planning used=%luB free_min=%luB cfg=%luB\r\n", used_bytes, free_bytes, cfg_bytes);
  }
#endif
}

void TargetSwitch(void const *argument)
{
  osSignalWait(Signal_DogInitOK, osWaitForever);
  /* Infinite loop */
  for (;;)
  {
    task_switch(&world.dog, &world.box_groups, &world.return_field);
    osDelay(1000);
  }
}

void Planning(void const *argument)
{
  osSignalWait(Signal_DogInitOK, osWaitForever);
  world.dog.plan.type = POINT_TO_POINT;
  world.dog.target_location.pos.x = 10.0f;
  world.dog.target_location.pos.y = 10.0f;
  ActionControl_InitWithDog(&world.dog);

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
