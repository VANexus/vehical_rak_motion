#!/bin/sh
# ---------------------------------------------------------------------------
# 共享工具链环境（Linux / macOS）。Windows 成员用同目录的 activate.ps1。
#
# 用法（在仓库任意位置都可以）：
#   . ./env/activate.sh          # bash / zsh 通用，注意前面的点
#
# 作用：把 STM32CubeCLT 的编译器 / cmake / ninja / 烧录工具前置到 PATH，
# 之后两个工程都用同一套命令：
#   cd vehical_rak_motion && cmake --preset Debug && cmake --build --preset Debug
#
# 只修改当前 shell 的 PATH，不写任何系统配置、不安装任何东西。
# ---------------------------------------------------------------------------

# 候选安装位置（按顺序探测）。用 set -- 存成位置参数，避开 zsh 不做变量分词的坑。
set -- \
    /opt/st/stm32cubeclt_1.22.0 \
    /opt/st/stm32cubeclt \
    /usr/local/st/stm32cubeclt_1.22.0 \
    "$HOME/STM32CubeCLT" \
    "$HOME/.local/share/st/stm32cubeclt"

# 允许用环境变量强行指定安装位置（优先级最高）
[ -n "$STM32CubeCLT_DIR" ] && set -- "$STM32CubeCLT_DIR" "$@"

CLT=""
for d in "$@"; do
    [ -x "$d/GNU-tools-for-STM32/bin/arm-none-eabi-gcc" ] && CLT="$d" && break
done

if [ -z "$CLT" ]; then
    echo "[env] 没找到 STM32CubeCLT。请安装后重试，或指定：" 1>&2
    echo "[env]   STM32CubeCLT_DIR=/你的/STM32CubeCLT_1.x . ./env/activate.sh" 1>&2
    return 1 2>/dev/null || exit 1
fi

for sub in GNU-tools-for-STM32/bin CMake/bin Ninja/bin Make/bin STM32CubeProgrammer/bin STLink-gdb-server/bin; do
    [ -d "$CLT/$sub" ] && PATH="$CLT/$sub:$PATH"
done

# ST 的 cube 启动器（cube mx = CubeMX 无 GUI CLI），存在就一并暴露
[ -d "$HOME/.local/stm32cube/bin" ] && PATH="$HOME/.local/stm32cube/bin:$PATH"

export PATH
export STM32CubeCLT_DIR="$CLT"

echo "[env] CubeCLT = $CLT"
echo "[env] cc      = $(command -v arm-none-eabi-gcc)  ($(arm-none-eabi-gcc -dumpversion))"
echo "[env] cmake   = $(command -v cmake)  ($(cmake --version | head -1 | cut -d' ' -f3))"
echo "[env] ninja   = $(command -v ninja)  ($(ninja --version))"
echo "[env] flash   = $(command -v STM32_Programmer_CLI)"
