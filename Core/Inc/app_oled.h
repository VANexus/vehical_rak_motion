#ifndef APP_OLED_H
#define APP_OLED_H

#include <stdint.h>

/* [AI]  SSD1306 OLED 调试显示（软 SPI：PD11=DC / PD12=RES / PD13=DAT / PD14=CLK）。
   R13 显示模块回归（R5 移除，用户决定加回用于调试）。惰性初始化：首次调用
   oled_show_step 才动 GPIO/时序，不占 boot 路径时间（R3 教训）。
   内容：电池/使能/复位原因、IMU 与自稳定诊断、四轮转速、车体速度、模式状态。 */

void oled_show_step(uint32_t now_ms);   /* [AI]  10Hz 任务里调；内部 5Hz 重绘 */

#endif /* APP_OLED_H */
