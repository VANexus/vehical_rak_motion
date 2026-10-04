/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "app_car.h"
#include "app_can.h"
#include "app_oled.h"
#include "app_servo.h"   /* [AI] 舵机自检 */
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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void task_control(void *argument);
static void task_can_rx(void *argument);
static void task_can_tx(void *argument);
static void task_status(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  static const osThreadAttr_t ctrl_attr = {
    .name = "ctrl", .stack_size = 1024, .priority = (osPriority_t) osPriorityHigh };
  static const osThreadAttr_t canrx_attr = {
    .name = "canrx", .stack_size = 1024, .priority = (osPriority_t) osPriorityAboveNormal };
  static const osThreadAttr_t cantx_attr = {
    .name = "cantx", .stack_size = 1024, .priority = (osPriority_t) osPriorityNormal };
  static const osThreadAttr_t stat_attr = {
    .name = "stat", .stack_size = 2048, .priority = (osPriority_t) osPriorityBelowNormal };
  osThreadNew(task_control, NULL, &ctrl_attr);   /* [AI]  100Hz 监督器 */
  osThreadNew(task_can_rx, NULL, &canrx_attr);   /* [AI]  CAN 接收解析 */
  osThreadNew(task_can_tx, NULL, &cantx_attr);   /* [AI]  1kHz 发送 */
  osThreadNew(task_status, NULL, &stat_attr);    /* [AI]  10Hz 喂狗 */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* [AI] 舵机自检：只扫 ch0 = PC6（约 3 秒一轮，循环）
     用途：确认底盘固件 + 舵机代码合并后能正常工作。
     ⚠️ 只在舵机没装到机械臂上时用；装好后要换成正经控制程序。
     ⚠️ 故意只扫 ch0——ch5=PB15 是板上阿克曼转向舵机口，全扫会误伤。 */
  for(;;)
  {
    servo_sweep_test(0);
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void task_control(void *argument)
{
  (void)argument;
  TickType_t last = xTaskGetTickCount();
  car_start_control_isr();   /* [AI]  R3: 25kHz ISR 延迟到调度器启动后开 */
  for (;;) {
    car_control_step(HAL_GetTick());
    vTaskDelayUntil(&last, pdMS_TO_TICKS(10));      /* 100 Hz */
  }
}

static void task_can_rx(void *argument)
{
  (void)argument;
  for (;;) {
    can_rx_task_step();                             /* [AI]  阻塞于队列 */
  }
}

static void task_can_tx(void *argument)
{
  (void)argument;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    can_tx_task_step(HAL_GetTick());
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));       /* 1 kHz */
  }
}

static void task_status(void *argument)
{
  (void)argument;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    car_feed_watchdog();                            /* [AI]  10Hz 喂狗 */
    oled_show_step(HAL_GetTick());                  /* [AI]  R13: OLED 调试显示(惰性初始化,内部 5Hz 重绘) */
    vTaskDelayUntil(&last, pdMS_TO_TICKS(100));     /* 10 Hz */
  }
}

/* USER CODE END Application */

