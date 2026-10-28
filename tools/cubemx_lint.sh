#!/usr/bin/env bash
# [AI] cubemx_lint.sh —— 外设配置越界检查（docs/CUBEMX-SOP.md 的机器化门禁）
#
# 目的：防止应用层重新出现手写的寄存器级外设初始化，以及生成物被改坏。
# 用法：bash tools/cubemx_lint.sh        （退出码 0 = 全过）
#
# 背景：R20 之前外设全是手写寄存器配置，代价是 ADC 采样时间写错寄存器、
# ADC 时钟超规格、CAN 位时序漏配、缺 TIM2/TIM5/USART3、多出 CAN2。
# 这份脚本把这些失败模式变成可执行的检查。

set -uo pipefail
cd "$(dirname "$0")/.." || exit 2

fail=0
ok()   { printf '  \033[32m✅\033[0m %s\n' "$1"; }
bad()  { printf '  \033[31m❌\033[0m %s\n' "$1"; fail=1; }
hdr()  { printf '\n\033[1m%s\033[0m\n' "$1"; }

APP_SRC=$(ls Core/Src/app_*.c 2>/dev/null)
[ -z "$APP_SRC" ] && { echo "找不到 Core/Src/app_*.c"; exit 2; }

# ---------------------------------------------------------------- 1. 应用层不得写外设寄存器
hdr "1. 应用层不得手写外设初始化"
# RCC 时钟使能、GPIO 模式/复用、定时器分频/重装载、CAN 位时序
PAT='RCC->(AHB1ENR|AHB2ENR|APB1ENR|APB2ENR)|->MODER|->OTYPER|->PUPDR|->AFR\[|TIM[0-9]+->(PSC|ARR|CCMR|CCER|SMCR|BDTR|CR1)|CAN[12]->(MCR|BTR|FMR)|USART[0-9]->(BRR|CR1)|ADC1->(CR1|CR2|SQR|SMPR)|IWDG->(KR|PR|RLR)'
HITS=$(grep -nE "$PAT" $APP_SRC 2>/dev/null)
if [ -z "$HITS" ]; then
    ok "app_*.c 里没有寄存器级外设初始化"
else
    bad "app_*.c 里出现寄存器级外设初始化（应改到 CubeMX 里配）:"
    echo "$HITS" | sed 's/^/      /'
fi

# ---------------------------------------------------------------- 2. 应用层不得定义中断向量
hdr "2. 应用层不得定义 *_IRQHandler（会与生成物重复符号）"
HITS=$(grep -nE '^[[:space:]]*(void|__weak[[:space:]]+void)[[:space:]]+[A-Za-z0-9_]+_IRQHandler[[:space:]]*\(' $APP_SRC 2>/dev/null)
if [ -z "$HITS" ]; then
    ok "app_*.c 里没有定义中断向量"
else
    bad "应用层定义了中断向量（应写成普通函数，从生成 handler 的 USER CODE 块调用）:"
    echo "$HITS" | sed 's/^/      /'
fi

# ---------------------------------------------------------------- 3. 生成物：三个 handler 不得回来
hdr "3. 生成物：stm32f4xx_it.c 不得恢复 SVC/PendSV/SysTick"
HITS=$(grep -nE '^[[:space:]]*void[[:space:]]+(SVC_Handler|PendSV_Handler|SysTick_Handler)[[:space:]]*\(' Core/Src/stm32f4xx_it.c 2>/dev/null)
if [ -z "$HITS" ]; then
    ok "三个 handler 未出现在生成物里（由 FreeRTOS 提供）"
else
    bad "这三个 handler 被 CubeMX 恢复了，会与 FreeRTOS 冲突:"
    echo "$HITS" | sed 's/^/      /'
fi

# ---------------------------------------------------------------- 4. 时钟源必须还是 HSE
hdr "4. 时钟源：SystemClock_Config 必须仍用 HSE 8 MHz"
if grep -qE 'RCC_OscInitStruct\.HSEState[[:space:]]*=[[:space:]]*RCC_HSE_ON' Core/Src/main.c 2>/dev/null; then
    ok "HSE 仍是时钟源（1 Mbps CAN 依赖它）"
else
    bad "SystemClock_Config 里的 HSE 被抹掉了（1 Mbps CAN 会失准）"
fi

# ---------------------------------------------------------------- 5. .ioc 里启用的外设都要有 MX_*_Init
hdr "5. .ioc 启用的每个外设都要有对应的 MX_*_Init() 调用"
[ -n "${IOC:-}" ] && printf "    (ioc: %s)\n" "$IOC"
# IP 名 -> main.c 里 Init 函数名的映射（多数是 MX_<IP>_Init，少数带后缀）
init_for() {
    case "$1" in
        USART[0-9]*) echo "MX_${1}_UART_Init" ;;
        *)           echo "MX_${1}_Init" ;;
    esac
}
# 选中本仓的 .ioc（两仓文件名不同：底盘 = vehical_rak_motion.ioc，机械臂 = vehical_rak_arm.ioc）。
# 规则：目录名同名者优先，其次唯一的 *.ioc；多个且无法判定则报错。
IOC=""
d=$(basename "$PWD")
[ -f "$d.ioc" ] && IOC="$d.ioc"
if [ -z "$IOC" ]; then
    cand=$(ls *.ioc 2>/dev/null | sort -u)
    n=$(echo "$cand" | grep -c .)
    if [ "$n" = "1" ]; then IOC="$cand"; fi
fi
if [ -z "$IOC" ]; then
    bad "找不到本仓唯一的 .ioc（用 $d.ioc 或唯一 *.ioc 判定失败）"
fi
IPS=$(grep -oE '^Mcu\.IP[0-9]+=.*' "$IOC" 2>/dev/null | cut -d= -f2 | sort -u)
INITS=$(grep -oE 'MX_[A-Z0-9_]+_Init\(\)' Core/Src/main.c 2>/dev/null | sed 's/()//' | sort -u)
missing=""
for ip in $IPS; do
    case "$ip" in
        NVIC|RCC|SYS|FREERTOS) continue ;;   # 无独立 Init
    esac
    want=$(init_for "$ip")
    echo "$INITS" | grep -qx "$want" || missing="$missing $ip($want)"
done
if [ -z "$missing" ]; then
    ok "全部外设都有 Init 调用（共 $(echo "$IPS" | wc -w) 个 IP）"
else
    bad "以下外设在 .ioc 里启用但 main.c 没有 Init（R20 就漏过这个）:$missing"
fi

# ---------------------------------------------------------------- 6. USER CODE 2 必须在所有 MX_*_Init 之后
hdr "6. main.c：USER CODE 2 块必须在所有外设 MX_*_Init() 之后"
# 注意排除 MX_FREERTOS_Init —— 它是调度器初始化，本来就排在 USER CODE 2 之后
L_LAST=$(grep -nE 'MX_[A-Z0-9_]+_Init\(\);' Core/Src/main.c 2>/dev/null \
         | grep -v 'MX_FREERTOS_Init' | tail -1 | cut -d: -f1)
L_U2=$(grep -n 'USER CODE BEGIN 2' Core/Src/main.c 2>/dev/null | cut -d: -f1)
if [ -n "$L_LAST" ] && [ -n "$L_U2" ] && [ "$L_U2" -gt "$L_LAST" ]; then
    ok "USER CODE 2 在第 $L_U2 行，最后一个外设 MX_*_Init 在第 $L_LAST 行"
else
    bad "USER CODE 2（行 ${L_U2:-?}）没有排在最后一个外设 MX_*_Init（行 ${L_LAST:-?}）之后——应用初始化会早于外设初始化"
fi

# ---------------------------------------------------------------- 7. MX_DMA_Init 必须在 USART Init 之前
hdr "7. main.c：MX_DMA_Init 必须在 MX_USART*_Init 之前"
L_DMA=$(grep -n 'MX_DMA_Init();' Core/Src/main.c 2>/dev/null | cut -d: -f1)
L_US=$(grep -nE 'MX_USART[0-9]+_UART_Init\(\);' Core/Src/main.c 2>/dev/null | head -1 | cut -d: -f1)
if [ -z "$L_US" ]; then
    ok "没有启用的 USART（跳过）"
elif [ -n "$L_DMA" ] && [ "$L_DMA" -lt "$L_US" ]; then
    ok "MX_DMA_Init(行 $L_DMA) 在 MX_USART*_Init(行 $L_US) 之前"
else
    bad "MX_DMA_Init（行 ${L_DMA:-缺失}）没有排在 MX_USART*_Init（行 $L_US）之前"
fi

# ---------------------------------------------------------------- 8. 生成物块外不得被改动（粗查）
hdr "8. 生成物：USER CODE 块数量是否与基线一致（粗查块外改动）"
# 只做温和提示：统计各生成物的 USER CODE 标记数，与 git HEAD 对比
for f in Core/Src/main.c Core/Src/freertos.c Core/Src/stm32f4xx_it.c; do
    now=$(grep -c 'USER CODE' "$f" 2>/dev/null || echo 0)
    old=$(git show HEAD:"$f" 2>/dev/null | grep -c 'USER CODE' || echo 0)
    if [ "$now" = "$old" ]; then
        ok "$f 的 USER CODE 标记数未变（$now）"
    else
        bad "$f 的 USER CODE 标记数变了（$old → $now）——生成物可能被重生成过，请核对块外内容"
    fi
done

# ---------------------------------------------------------------- 汇总
printf '\n'
if [ "$fail" -eq 0 ]; then
    printf '\033[32m[cubemx_lint] 全部通过\033[0m\n'
else
    printf '\033[31m[cubemx_lint] 有违规项 —— 见 docs/CUBEMX-SOP.md\033[0m\n'
fi
exit "$fail"
