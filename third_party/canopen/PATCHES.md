# third_party/canopen —— 上游来源与本仓改动

| 组件 | 上游 | 版本固定 | 许可 |
| --- | --- | --- | --- |
| `CANopenNode/` | github.com/CANopenNode/CANopenNode（本仓经 gitcode 镜像 clone） | v4 @ `5a97d2d20962512823015bc12452d90c423c9f73`（2026-07-08） | Apache-2.0 |
| `CANopenNode_STM32/` | github.com/CANopenNode/CanOpenSTM32 | mirror clone 2026-10-29 | Apache-2.0 |

两块 MCU 仓（`vehical_rak_motion` / `vehical_rak_edge_ai`）持有**同一份**副本，
`docs/canopen/*.eds` + 生成的 `Core/{Inc/OD.h,Src/OD.c}` 按节点分别生成。
CI 用 `sha256sum third_party/canopen/CANopenNode/CANopen.c` 等比对两仓一致性。

## 改动清单（每一项都是必须的，不做等于跑不起来）

1. **`CANopenNode_STM32/CO_app_STM32.c`：日志宏改为空操作**
   `log_printf(...)` → `((void)0)`。本工程无控制台；避免把 newlib `printf` 链进固件。

2. **`CANopenNode_STM32/CO_app_STM32.c`：`timerHandle` 允许为 NULL**
   `HAL_TIM_Base_Start_IT/Stop_IT` 加 NULL 判断。1 ms 节拍由 FreeRTOS 任务
   （底盘 `task_can_tx`、机械臂 `task_can_tx`）调用 `canopen_app_interrupt()` 提供，
   不额外占用 CubeMX 定时器，也不覆盖 `HAL_TIM_PeriodElapsedCallback`。

3. **`CANopenNode_STM32/CO_app_STM32.c`：上电不进 Operational**
   `NMT_CONTROL` 去掉 `CO_NMT_STARTUP_TO_OPERATIONAL`。安全要求：节点必须停在
   Pre-operational，等 Jetson 显式 NMT Start 才允许 PDO 收发（设计规格 §2、§5）。

4. **`CANopenNode_STM32/CO_driver_STM32.c`：RX 软件过滤器改为正向遍历（关键 bug 修复）**
   上游（含 blank 驱动）从 `rxArray` 尾部往前匹配。OD 里 8 个未使用的
   Consumer heartbeat time（0x1016）槽位会注册成 `ident=0x000 / mask=0x87FF`，
   于是 **NMT 报文（COB-ID 0x000）被 HB consumer 槽位先匹配掉**，NMT Start 永不生效，
   节点卡在 Pre-operational（SWD 实测：`CO_NMT_receive` 命中计数 3、NMT 状态仍 0x7F）。
   注册顺序本身就是「专用对象在前」（NMT, SYNC, EMCY, SDO, RPDO, HB, LSS），
   正向匹配即为正确语义。

## 使用约束（写进 EDS 生成器，不靠人记）

- **未使用的 RPDO/TPDO 必须把 COB-ID 的 bit31（transmission disabled）置 1**。
  否则 CANopenNode 视其为「有合法 COB-ID 但零映射」→ 报 `CO_EM_PDO_WRONG_MAPPING`
  → Error register 置位 → 因 `CO_NMT_ERR_ON_ERR_REG` 把节点从 Operational 踢回
  Pre-operational（实测：TPDO 全灭、心跳恒 0x7F）。`tools/canopen/gen_eds.py` 已强制生成。
- PDO COB-ID 在 EDS 里写**基础值**（0x180/0x280/…、0x200/0x300/…），由
  `CO_CANopenInitPDO()` 按 node ID 补偏移；写 `0x80000200+$NODEID` 这类禁用形式会
  让 PDO 静默不工作。

## OD 生成流程（不用手改生成物）

```bash
python3 tools/canopen/gen_eds.py docs/canopen/<node>.json \
         tools/canopen/template.eds docs/canopen/<node>.eds
dotnet EDSSharp.dll --infile docs/canopen/<node>.eds --outfile OD.c --type CanOpenNodeV4
cp OD.c Core/Src/OD.c; cp OD.h Core/Inc/OD.h
```
单一来源 = `docs/canopen/<node>.json`（对象/PDO 清单）→ EDS（CiA 306 标准格式）→
官方 CANopenEditor 导出器生成 `OD.c/h`。CANopenEditor 需 .NET（本机 `dotnet-sdk-10.0` +
`DOTNET_ROLL_FORWARD=Major` 跑 net8.0 CLI）；CI 未装该环境时，用 `gen_eds.py` +
提交生成的 `OD.c/h` 并在评审里核对 EDS 与 OD 一致。
