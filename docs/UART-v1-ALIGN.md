# F407 底盘固件侧 UART v1 对齐响应（R16）

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
