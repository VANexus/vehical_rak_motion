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
#include "app_dbg.h"
#include "app_buzz.h"   /* [AI] R66 */
#include "app_bt.h"      /* [AI] R67 */
#include "app_car.h"
#include "app_can.h"
#include "app_canopen.h"
#include "app_oled.h"
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
    .name = "ctrl", .stack_size = 4096, .priority = (osPriority_t) osPriorityHigh };
  static const osThreadAttr_t canrx_attr = {
    .name = "canrx", .stack_size = 4096, .priority = (osPriority_t) osPriorityAboveNormal };
  static const osThreadAttr_t cantx_attr = {
    .name = "cantx", .stack_size = 4096, .priority = (osPriority_t) osPriorityNormal };
  static const osThreadAttr_t stat_attr = {
    .name = "stat", .stack_size = 4096, .priority = (osPriority_t) osPriorityBelowNormal };
  /* [AI]  创建失败 = 堆不足。静默跑下去的后果是 25kHz ISR 永不启动
     （car_start_control_isr 在 ctrl 线程首拍），而板子看起来"活着"——
     这是最难查的失败模式，所以显式兜底。 */
  osThreadId_t h_ctrl  = osThreadNew(task_control, NULL, &ctrl_attr);  /* [AI]  100Hz 监督器 */
  osThreadId_t h_canrx = osThreadNew(task_can_rx, NULL, &canrx_attr);  /* [AI]  CAN 接收解析 */
  osThreadId_t h_cantx = osThreadNew(task_can_tx, NULL, &cantx_attr);  /* [AI]  1kHz 发送 */
  osThreadId_t h_stat  = osThreadNew(task_status, NULL, &stat_attr);   /* [AI]  10Hz 喂狗 */
  if (defaultTaskHandle == NULL || h_ctrl == NULL || h_canrx == NULL ||
      h_cantx == NULL || h_stat == NULL) {
    Error_Handler();
  }
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
  BOOT_MARK(6);
  (void)argument;   /* [AI]  未使用（其余四个任务同款写法），-Wextra 要求显式 */
  /* [AI] R41 舵机测试结束：defaultTask 恢复空转；R40 的极限互切测试代码
     已整体移除（R29~R40 实测结论记录在 git 历史 46ee418..本提交）。
     实测标定结论：ch0=PC6=J6/ch2=PC8=J4/ch5=PB15=J1 舵机、
     ch1=PC7=J5 与 ch3=PC9=J3 为 0~270° 位置型、ch4=PB14=J2 为 0~135° 机械限位，
     MG995/996R 500µs=0°、2500µs=270°；J4 已换 996R 位置型（行程待复核）。 */
  for(;;)
  {
    osDelay(1000);
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
    uint32_t now = HAL_GetTick();
    app_canopen_step(now);                           /* v5: CANopen 命令/反馈桥接 */
    car_control_step(now);
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
    uint32_t now = HAL_GetTick();
    can_tx_task_step(now);
    buzz_tick(now);                                 /* [AI] R66: 蜂鸣器状态机 */
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));       /* 1 kHz */
  }
}

static void task_status(void *argument)
{
  BOOT_MARK(7);
  (void)argument;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    uint32_t now = HAL_GetTick();
    car_feed_watchdog();                            /* [AI]  10Hz 喂狗 */
    oled_show_step(now);                            /* [AI]  R13: OLED 调试显示(惰性初始化,内部 5Hz 重绘) */
    bt_step(now);                                   /* [AI] R67: 蓝牙 1 Hz 遥测（只发不收） */
    vTaskDelayUntil(&last, pdMS_TO_TICKS(100));     /* 10 Hz */
  }
}

/* [AI]  ---- FreeRTOS 失败钩子（configCHECK_FOR_STACK_OVERFLOW=2 / malloc hook）----
   板子上没有控制台。做法：把现场留成 SWD 可直读的全局量，然后关中断空转，
   让 IWDG(2s) 复位。复位原因 'I'(IWDGRSTF) 会显示在 OLED 的 P 字段上
   （car_reset_cause），所以"反复显示 P I"= 有任务栈溢出或堆耗尽。 */
volatile char     dbg_overflow_task[configMAX_TASK_NAME_LEN];
volatile uint32_t dbg_overflow_cnt;
volatile uint32_t dbg_malloc_fail_cnt;

/* 签名按 FreeRTOS 侧的 char*（cmsis_os2.c 里那份 __WEAK 空实现是
   signed char*，属 ST 的笔误；强符号覆盖它即可）。 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  dbg_overflow_cnt++;
  for (unsigned i = 0U; i < configMAX_TASK_NAME_LEN; ++i) {
    dbg_overflow_task[i] = pcTaskName[i];
    if (pcTaskName[i] == '\0') break;
  }
  taskDISABLE_INTERRUPTS();
  for (;;) { }
}

void vApplicationMallocFailedHook(void)
{
  dbg_malloc_fail_cnt++;
  taskDISABLE_INTERRUPTS();
  for (;;) { }
}

/* USER CODE END Application */

