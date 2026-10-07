/* [AI] R38: encoder_sign kick 学习结果持久化（Flash 末扇区 append-only）。
 *
 * 背景：R32/R33 恢复 kick 自检后，每次上电都要重新学极性——学习期输出门
 * 75% kick 会让轮子跳一拍，且未学满前部分轮无闭环（精度与安全都是坑）。
 * 本模块把学到的 4 路极性写入 Flash 扇区 11（0x080E0000，128KB，与代码区
 * 无重叠——app 实测仅用 0x08013210 以下），下次上电直接预载。
 *
 * 擦除策略（关键安全设计）：
 *   128KB 大扇区整片擦除在运行中执行会把指令总线卡住 ~1-2s，25kHz ISR
 *   直接饿死。因此擦除只允许发生在 car_init（25kHz 未启动、电机未驱动）；
 *   运行期只做 16B 槽位 append 编程（doubleword 写 ~100µs 总线停顿，
 *   kick 学习完成点本就非实时敏感，可接受）。
 *
 * 记录布局（16B 对齐槽，1024 槽 = 16KB…用满后下次开机擦）：
 *   [0..3]  magic 0x53474E31 ("SNG1")
 *   [4..7]  seq u32 单调递增（取最高 seq 为最新）
 *   [8..11] signs[4]（int8_t，±1；0 视为无效记录）
 *   [12..15] crc32 前 12 字节
 */
#ifndef APP_SIGN_STORE_H
#define APP_SIGN_STORE_H

#include <stdint.h>

/* [AI] R38 联测修正：F407VET6 = 512KB Flash（0x08000000-0x0807FFFF）。
 * 扇区布局：0-3=16KB、4=64KB、5-11=128KB → 扇区11 = 0x08060000（末扇区）。
 * 最初误按 1MB 的 VGT6 写 0x080E0000——超出芯片范围 → PGAERR+PGSERR（实测）。
 * app 实测仅用 0x08013210 以下，末扇区与代码区无重叠。 */
#define SIGN_STORE_MAGIC   0x53474E31UL
#define SIGN_STORE_SLOT_N  8192U   /* 128KB / 16B */
#define SIGN_STORE_SECTOR  FLASH_SECTOR_11
#define SIGN_STORE_BASE    0x08060000UL

/* [AI] 开机调用（25kHz/调度器启动前）：扇区满则整片擦除。
 * 返回 0 成功；非 0 见实现（Flash 操作失败）。 */
uint32_t sign_store_prepare(void);

/* [AI] 有效记录则填充 signs_out 并返回 1；无记录返回 0。 */
uint8_t sign_store_load(int8_t signs_out[4]);

/* [AI] 追加一条记录（运行期，任务上下文）。返回 0 成功。 */
uint32_t sign_store_save(const int8_t signs[4]);

#endif /* APP_SIGN_STORE_H */
