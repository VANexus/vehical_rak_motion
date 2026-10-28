# F407 底盘固件侧 UART v1 对齐响应（R16）

> ⚠️ **历史备用链路（2026-10-28）**：本文的 USB v1 是**单板时期**的链路。
> 当前 **CAN v4 是唯一主链路**；本文仅作历史与 USB 备用路径参考。

回应 `rak 仓 docs/protocol/PROTOCOL_ALIGN_PACKET.md` §8 的 7 项确认。
状态：**v1 已实现并烧 A 板**（build_tag R16，commit 见 git log），v1.1 暂不实现。
实现在 `Core/Src/app_serial.c`（COBS 解码于 USART IRQ 逐字节跑，TX 走 TXE 中断续发）。

## §8 七项确认

1. ✅ **物理链路**：921600 8N1 USB-UART 不变，指令/遥测各 100Hz。
2. ✅ **v1 全量实现**：COBS+CRC16/CCITT-FALSE 帧格式、HELLO/HELLO_ACK/
   SET_BASE_VELOCITY/BASE_STATE/FAULT_EVENT 五种消息、序号规则
   ((cand-prev)mod65536 ∈ 1..32767，HELLO 重置)、50ms lease 看门狗、
   DISARMED 复位行为、状态回显。REQ_FEATURES = bit0|1。
3. ⏸ **feature bit 3（v1.1 原子动作）暂不报**——host 默认自动回退 v1；
   待 v1 台架（Stage1-3）通过后再增量实现。
4/5/6. ⏸ 跟随第 3 项（v1.1 延后）。
7. ✅ **故障位实现**（对齐包 fault_bitmap bit0-7）：
   - bit0 E-stop：CAN SAFETY(0x010)/0x102 急停锁存，CLEAR_ESTOP 清除
   - bit1 lease 超时：锁存，**新 HELLO 清除**（恢复通信不自动 ARM 不变）
   - bit2 CAN 对端离线：**台架让路版**——收到过 0x031 后静默 >500ms 才置位；
     arm 固件到位后改为"上电即判"
   - bit3 对端故障：0x031 的 FAULT_CODE≠0
   - bit4 电机驱动：底盘 FAULT 模式（台架跑失败等）
   - bit5 欠压：<10500 mV（3S，可调）
   - bit6 过温：板上无温度传感器，恒 0（N/A）
   - bit7 反馈无效：预留恒 0（判定标准待台架定标）
   - ARM 条件：已握手 && fault_bitmap==0 && mode=1；任一故障位置位即 DISARM

## 实现注记

- **单位换算**：host `wz_mrad_s`（i32 LE）→ 固件内部 0.01 rad/s（÷10）；
  BASE_STATE 回读 ×10 还原 mrad/s。vx/vy mm/s 两边一致。
- **HELLO 附带 DISARM**（保守设计）；序号状态随 HELLO 重置。
- **boot_id**：每次上电取 CYCCNT 混合生成，必变。
- **识别横幅**：`ID:A\n` @1Hz 保留（对齐包 §1 已采纳；换 B 板编译 `BOARD_ID="B"`）。
- **旧协议**：8 字节 CRC8 实验帧已整体移除，串口通道只讲 v1。
- **CAN 侧不变**：0x100/0x200/0x030/0x031 照旧（Jetson 不在 CAN 总线），
  v1 的速度指令与 CAN 0x100 汇入同一条 car_can_velocity 路径。

## 验收状态

- Stage1（PTY/镜像）：Python 镜像工具 `tools/uart_v1_test.py` 自检通过
  （CRC16 0x29B1 校验值 + HELLO golden vector 逐字节一致 + 编解码回环）。
  固件↔host 联调待 A 板 USB 枚举恢复后进行。
- Stage2-5：待 USB 链路恢复后按对齐包 §7 顺序推进。

---

## 实测补充（2026-10-05 深夜，R17/R18 实车数据）

### 链路质量实测
- 实测 RX 带宽 **91514 B/s = 理论 99.3%**（921600 8N1），控制+遥测仅占 6.5%，余量 15 倍
- 端到端：握手/HELLO_ACK/BASE_STATE 全通，echo 100Hz 零丢帧（工具 `tools/uart_v1_test.py`，golden vector 自检过）

### ⚠️ 需要团队签字的偏离：lease 200ms
- **现象**：电机 PWM 运行时 USART 溢出(ORE)成簇，字节级丢失→帧丢失，**50ms lease 必触发超时锁存**（架空+落地 100% 复现）
- **数据**：R17 诊断计数器（v1_dbg_ore 等）实锤；ORE 根因=串口 IRQ 优先级 7 被 RTOS 临界区推迟（R18 已提至 4，ORE 25→4，-84%）
- **当前**：固件接受 lease ≤500（R17 临时放宽），host 实发 200ms——**行驶不再卡顿，但偏离规范 [10,50]**
- **请求**：①批准 200ms 作为台架/实车过渡值；②物理层整治（串口线远离电机线/短/双绞/磁环）后回到 50ms
- **对 Jetson host 的影响**：node.py 的 lease 处理需兼容（若保持 50ms，实车驱动会复现同样卡顿）

### 已知现象（待查）
- 板子从开机起持续收到 ~2000 B/s 的线级流量/噪声（ORE 计数速率反推）——来源未明（串口线悬空噪声？CH9102 行为？），整治前建议逻辑分析仪抓一次
- DMA 收据改造（R19 尝试）**已回滚**：寄存器配置肉眼全对但 DMA 不搬运（ORE 暴涨、NDTR 冻结），需逻辑分析仪或改用 ST HAL 抽象重做，详见 git 历史/交接文档

### host 工具（本次入库 tools/）
- `chassis_host.py`：Python 交互控制类（car.forward/back/rotate/turn/strafe/move_mm/stop/status），后台线程 100Hz+lease，故障自动刹停+重握手
- `uart_v1_test.py`：协议镜像测试（golden vector 自检+握手+遥测统计）

---

## R19 实现变更（2026-10-06）

三个**内部**缺陷修复：**协议不变、wire 格式不变、host 侧无需任何改动**。
（同批还改了 ADC 采样/时钟配置与看门狗钩子，与串口无关，见 git log。）

1. **HELLO_ACK 移出中断**。原先在 USART IRQ 里直接调 `v1_send()`，而 `v1_send()`
   用静态编帧缓冲，与任务侧的 BASE_STATE / FAULT_EVENT 共用 —— USART 优先级 4
   高于 FreeRTOS 临界区阈值 5，可随时抢占任务，两块缓冲会互相覆写。
   现改为 IRQ 只置标志，`serial_v1_tick()` 在任务上下文发。
   **代价：HELLO_ACK 最坏延迟 1 ms**（1kHz 节拍），host 的重试逻辑不受影响。
2. **COBS 解码状态按端口各一份**。原先 USART1/USART3 共用一份解码状态，两路
   字节交错会互相撕帧（悬空端口的线级噪声尤其容易触发——可能与"开机起
   ~2000 B/s 噪声"那条待查现象相关）。协议层状态（`v1_armed` 等）仍是单份：
   **同一时刻只应有一个 host 在讲话**。
3. **TIM6 中断的临界区**。`reset_positions()` 原先用 `taskENTER_CRITICAL()`，
   但 TIM6 ISR 优先级 2 高于 `configMAX_SYSCALL_INTERRUPT_PRIORITY`(=5)，
   BASEPRI 屏蔽不到它，临界区是假的。改为直接关 TIM6 的 NVIC 使能。

另：`configCHECK_FOR_STACK_OVERFLOW=2` 与 malloc 失败钩子已开启。栈溢出/堆耗尽
会关中断空转 → IWDG 复位，OLED 的 `P` 字段显示 `I`。
**反复复位且 P 字段为 `I`，就是这个原因**（现场量在 `dbg_overflow_task` /
`dbg_overflow_cnt` / `dbg_malloc_fail_cnt`，SWD 直读）。
