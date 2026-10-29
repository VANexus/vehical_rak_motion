#ifndef APP_DBG_H
#define APP_DBG_H
#include <stdint.h>
/* [AI] 启动阶段标记：每一步写一个数，SWD 直接读 dbg_boot_stage 就知道卡在哪。
   1=main 进入  2=外设/MX init 完成  3=app_init(car/arm) 完成  4=osKernelInitialize
   5=MX_FREERTOS_Init 完成  6=StartDefaultTask 进入  7=task_status 首次执行
   8=oled_init 进入  9=oled_init 完成  10=第一帧 flush 完成 */
extern volatile uint32_t dbg_boot_stage;
#define BOOT_MARK(v) do { dbg_boot_stage = (uint32_t)(v); } while (0)
#endif
