# ---------------------------------------------------------------------------
# 共享工具链环境（Windows PowerShell）。Linux/macOS 成员用同目录 activate.sh。
#
# 用法：
#   . .\env\activate.ps1        # 注意前面的点（dot-source）
#
# 作用：把 STM32CubeCLT 的编译器 / cmake / ninja / 烧录工具前置到 $env:PATH，
# 之后两个工程都用同一套命令（与 Linux 完全一致）：
#   cd vehical_rak_motion; cmake --preset Debug; cmake --build --preset Debug
#
# ⚠️ 本脚本在 Linux 上编写，未在 Windows 实机验证过；Windows 成员首次执行请核对
#    打印出来的路径与版本。探测顺序若与你的安装位置不符，直接传参指定：
#   . .\env\activate.ps1 -CltDir 'D:\STM\STM32CubeCLT_1.22.0'
# ---------------------------------------------------------------------------

param([string]$CltDir)

$ErrorActionPreference = 'Stop'

# 候选安装位置，按优先级排列（CubeCLT 默认装到 C:\st\STM32CubeCLT_<版本>）
$candidates = @()
if ($CltDir) { $candidates += $CltDir }
$candidates += @(
    'C:\st\STM32CubeCLT_1.22.0',
    'C:\st\STM32CubeCLT',
    'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeCLT',
    'C:\Program Files\STMicroelectronics\STM32CubeCLT',
    'D:\st\STM32CubeCLT_1.22.0',
    'D:\STM\STM32CubeCLT_1.22.0'
)

$clt = $null
foreach ($d in $candidates) {
    if (Test-Path -LiteralPath (Join-Path $d 'GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe')) {
        $clt = $d; break
    }
}

if (-not $clt) {
    Write-Host '[env] 没找到 STM32CubeCLT（缺 GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe）。' -ForegroundColor Red
    Write-Host '[env] 装 CubeCLT，或用 -CltDir 指定安装目录后重试。' -ForegroundColor Red
    return
}

$subs = 'GNU-tools-for-STM32\bin', 'CMake\bin', 'Ninja\bin', 'Make\bin',
        'STM32CubeProgrammer\bin', 'STLink-gdb-server\bin'
foreach ($s in $subs) {
    $p = Join-Path $clt $s
    if (Test-Path -LiteralPath $p) { $env:PATH = "$p;$env:PATH" }
}
$env:STM32CubeCLT_DIR = $clt

Write-Host "[env] CubeCLT = $clt"
Write-Host "[env] cc      = $((Get-Command arm-none-eabi-gcc.exe -ErrorAction SilentlyContinue).Source)"
Write-Host "[env] cmake   = $((Get-Command cmake.exe -ErrorAction SilentlyContinue).Source)"
Write-Host "[env] ninja   = $((Get-Command ninja.exe -ErrorAction SilentlyContinue).Source)"
Write-Host "[env] flash   = $((Get-Command STM32_Programmer_CLI.exe -ErrorAction SilentlyContinue).Source)"

# 台架归档区（car2/）的旧裸机脚本用的是自带的 12.2.1 工具链和 PowerShell 脚本，
# 与上面的 CubeCLT 无关；只有在跑 car2 里的 build.ps1 时才需要额外加载。
#   . ..\car2\tools\activate.ps1
Write-Host '[env] 提示：编译 car2/ 下的裸机实验请改用 car2/tools/activate.ps1。' -ForegroundColor DarkGray
