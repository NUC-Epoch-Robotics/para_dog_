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
#include "menu.h"
#include "key.h"
#include "remote.h"
#include "decode.h"
#include "menu_left.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern int16_t joy_x ;
extern int16_t joy_y ;

extern int16_t joy_x_left;
extern int16_t joy_y_left;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern frame_parser_t parser_t;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for menutask */
osThreadId_t menutaskHandle;
const osThreadAttr_t menutask_attributes = {
  .name = "menutask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for joytask */
osThreadId_t joytaskHandle;
const osThreadAttr_t joytask_attributes = {
  .name = "joytask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for UART2_RX */
osThreadId_t UART2_RXHandle;
const osThreadAttr_t UART2_RX_attributes = {
  .name = "UART2_RX",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for myTimer01 */
osTimerId_t myTimer01Handle;
const osTimerAttr_t myTimer01_attributes = {
  .name = "myTimer01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void menutask1(void *argument);
void joytask_(void *argument);
void StartTask03(void *argument);
void Callback01(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	parser_init(&parser_t);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of myTimer01 */
  myTimer01Handle = osTimerNew(Callback01, osTimerPeriodic, NULL, &myTimer01_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of menutask */
  menutaskHandle = osThreadNew(menutask1, NULL, &menutask_attributes);

  /* creation of joytask */
  joytaskHandle = osThreadNew(joytask_, NULL, &joytask_attributes);

  /* creation of UART2_RX */
  UART2_RXHandle = osThreadNew(StartTask03, NULL, &UART2_RX_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_menutask1 */
/**
  * @brief  Function implementing the menutask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_menutask1 */
void menutask1(void *argument)
{
  /* USER CODE BEGIN menutask1 */
  /* Infinite loop */
  for(;;)
  {	
	if (Key_WasPressed(KEY_9)) Menu_Keyup();
	if (Key_WasPressed(KEY_13)) Menu_KeyDowm();
	if (Key_WasPressed(KEY_10)) Menu_KeyOK();
	  
	if (Key_WasPressed(KEY_5)) Menu_Keyup_left();
	if (Key_WasPressed(KEY_7)) Menu_KeyDowm_left();
	if (Key_WasPressed(KEY_6)) Menu_KeyOK_left();
	  
	Menu_Draw();
	Menu_Draw_left();
	  
    osDelay(10);
  }
  /* USER CODE END menutask1 */
}

/* USER CODE BEGIN Header_joytask_ */
/**
* @brief Function implementing the joytask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_joytask_ */
void joytask_(void *argument)
{
  /* USER CODE BEGIN joytask_ */
  /* Infinite loop */
  for(;;)
  {
	  joy_x = joy_get_x();
	  joy_y = joy_get_y();
	  
	  joy_x_left = joy_get_x_left();
	  joy_y_left = joy_get_y_left();
    osDelay(1);
  }
  /* USER CODE END joytask_ */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the UART2_RX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
	usart_start_rx_dma();
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */

  /* USER CODE END Callback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

