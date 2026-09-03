param(
    [string]$Compiler = "gcc.exe"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$testDir = Join-Path $repoRoot "tests/host"
$legacyDir = Join-Path $repoRoot "tests/legacy"
$focFakeIncludeDir = Join-Path $testDir "fakes"
$adapterDir = Join-Path $repoRoot "firmware/AxDr_App/User/adapter"
$appDir = Join-Path $repoRoot "firmware/AxDr_App/User/app"
$bspDir = Join-Path $repoRoot "firmware/AxDr_App/User/bsp"
$bspIncludeDir = Join-Path $bspDir "inc"
$commonDir = Join-Path $repoRoot "firmware/AxDr_App/User/common"
$configDir = Join-Path $repoRoot "firmware/AxDr_App/User/config"
$controlDir = Join-Path $repoRoot "firmware/AxDr_App/User/control"
$diagnosticIncludeDir = Join-Path $repoRoot "firmware/AxDr_App/User/diagnostic/include"
$motorDir = Join-Path $repoRoot "firmware/AxDr_App/User/motor"
$driveDir = Join-Path $repoRoot "firmware/AxDr_App/User/drive"
$focTestSource = Join-Path $testDir "test_foc_math.c"
$driveModeTestSource = Join-Path $testDir "test_drive_mode.c"
$driveDiagTestSource = Join-Path $testDir "test_drive_diag.c"
$driveCommandTestSource = Join-Path $testDir "test_drive_command.c"
$driveProtectionTestSource = Join-Path $testDir "test_drive_protection.c"
$driveResetTestSource = Join-Path $testDir "test_drive_reset.c"
$driveTestSource = Join-Path $testDir "test_drive_state.c"
$drivePwmTestSource = Join-Path $testDir "test_drive_pwm.c"
$pidTestSource = Join-Path $testDir "test_control_pid.c"
$cascadeTestSource = Join-Path $testDir "test_control_cascade.c"
$controlLoopTestSource = Join-Path $testDir "test_control_loop.c"
$debugSnapshotTestSource = Join-Path $testDir "test_debug_snapshot.c"
$controlCycleTestSource = Join-Path $testDir "test_control_cycle.c"
$controlReplayTestSource = Join-Path $testDir "test_control_replay.c"
$fastLoopTestSource = Join-Path $testDir "test_fast_loop.c"
$boardAdapterTestSource = Join-Path $testDir "test_board_adapter.c"
$publicHeadersTestSource = Join-Path $testDir "test_public_headers.c"
$controlCycleSource = Join-Path $appDir "control_cycle.c"
$fastLoopSource = Join-Path $appDir "fast_loop.c"
$targetIrqSource = Join-Path $bspDir "target_irq.c"
$boardAdapterSource = Join-Path $adapterDir "board_adapter.c"
$debugSnapshotSource = Join-Path $appDir "debug_snapshot.c"
$legacyFocSource = Join-Path $legacyDir "legacy_foc.c"
$legacyFocCoreSource = Join-Path $legacyDir "legacy_foc_core.c"
$legacyCascadeSource = Join-Path $legacyDir "legacy_control_cascade.c"
$legacyControlLoopSource = Join-Path $legacyDir "legacy_control_loop.c"
$legacyPidSource = Join-Path $legacyDir "legacy_pid.c"
$legacyUtilSource = Join-Path $legacyDir "legacy_util.c"
$filterSource = Join-Path $controlDir "control_filter.c"
$cascadeSource = Join-Path $controlDir "control_cascade.c"
$controlLoopSource = Join-Path $controlDir "foc_control.c"
$controlMitSource = Join-Path $controlDir "control_mit.c"
$focCoreSource = Join-Path $controlDir "foc_core.c"
$limitSource = Join-Path $controlDir "control_limit.c"
$svmSource = Join-Path $controlDir "foc_svm.c"
$transformSource = Join-Path $controlDir "foc_transform.c"
$controlPidSource = Join-Path $controlDir "control_pid.c"
$speedSource = Join-Path $controlDir "control_speed.c"
$utilSource = Join-Path $motorDir "util.c"
$driveSource = Join-Path $driveDir "drive.c"
$driveDiagSource = Join-Path $driveDir "drive_diag.c"
$driveModeSource = Join-Path $driveDir "drive_mode.c"
$driveCommandSource = Join-Path $driveDir "drive_command.c"
$driveProtectionSource = Join-Path $driveDir "drive_protection.c"
$driveResetSource = Join-Path $driveDir "drive_reset.c"
$drivePwmSource = Join-Path $driveDir "drive_pwm.c"
$outputDir = Join-Path $repoRoot "firmware/AxDr_App/build/host-tests"
$focExecutablePath = Join-Path $outputDir "test_foc_math.exe"
$driveModeExecutablePath = Join-Path $outputDir "test_drive_mode.exe"
$driveDiagExecutablePath = Join-Path $outputDir "test_drive_diag.exe"
$driveCommandExecutablePath = Join-Path $outputDir "test_drive_command.exe"
$driveProtectionExecutablePath = Join-Path $outputDir "test_drive_protection.exe"
$driveResetExecutablePath = Join-Path $outputDir "test_drive_reset.exe"
$driveExecutablePath = Join-Path $outputDir "test_drive_state.exe"
$drivePwmExecutablePath = Join-Path $outputDir "test_drive_pwm.exe"
$pidExecutablePath = Join-Path $outputDir "test_control_pid.exe"
$cascadeExecutablePath = Join-Path $outputDir "test_control_cascade.exe"
$controlLoopExecutablePath = Join-Path $outputDir "test_control_loop.exe"
$debugSnapshotExecutablePath = Join-Path $outputDir "test_debug_snapshot.exe"
$controlCycleExecutablePath = Join-Path $outputDir "test_control_cycle.exe"
$controlReplayExecutablePath = Join-Path $outputDir "test_control_replay.exe"
$fastLoopExecutablePath = Join-Path $outputDir "test_fast_loop.exe"
$boardAdapterExecutablePath = Join-Path $outputDir "test_board_adapter.exe"
$publicHeadersExecutablePath = Join-Path $outputDir "test_public_headers.exe"

$compilerCommand = Get-Command $Compiler -ErrorAction SilentlyContinue
if ($null -eq $compilerCommand) {
    throw "未找到电脑端 C 编译器。请把 gcc.exe 加入 PATH，或通过 -Compiler 指定路径。"
}

New-Item -ItemType Directory -Path $outputDir -Force | Out-Null

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        "-I$focFakeIncludeDir" "-I$commonDir" "-I$controlDir" "-I$motorDir" `
        "-I$driveDir" "-I$legacyDir" `
        $focTestSource $legacyFocSource $legacyFocCoreSource $legacyUtilSource `
        $filterSource $limitSource $speedSource $focCoreSource `
        $svmSource $transformSource $utilSource `
        -o $focExecutablePath -lm 2>&1 |
        ForEach-Object { $_.ToString() }
)
$compileExitCode = $LASTEXITCODE

if ($compileExitCode -ne 0) {
    $compileOutput | ForEach-Object { Write-Host $_ }
    throw "Host FOC 数学测试编译失败，退出码：$compileExitCode"
}

& $focExecutablePath
if ($LASTEXITCODE -ne 0) {
    throw "Host FOC 数学测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/FOC 数学测试通过。"

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        -Wno-misleading-indentation `
        "-I$focFakeIncludeDir" "-I$commonDir" "-I$controlDir" "-I$motorDir" `
        "-I$driveDir" "-I$legacyDir" `
        $pidTestSource $legacyPidSource $controlPidSource `
        -o $pidExecutablePath 2>&1 |
        ForEach-Object { $_.ToString() }
)
$compileExitCode = $LASTEXITCODE

if ($compileExitCode -ne 0) {
    $compileOutput | ForEach-Object { Write-Host $_ }
    throw "Host PID/PDFF 对照测试编译失败，退出码：$compileExitCode"
}

& $pidExecutablePath
if ($LASTEXITCODE -ne 0) {
    throw "Host PID/PDFF 对照测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/PID 与 PDFF 逐位对照测试通过。"

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        -Wno-misleading-indentation `
        "-I$focFakeIncludeDir" "-I$commonDir" "-I$controlDir" "-I$motorDir" `
        "-I$driveDir" "-I$legacyDir" `
        $cascadeTestSource $legacyCascadeSource $legacyPidSource $legacyUtilSource `
        $cascadeSource $controlPidSource $limitSource `
        -o $cascadeExecutablePath 2>&1 |
        ForEach-Object { $_.ToString() }
)
$compileExitCode = $LASTEXITCODE

if ($compileExitCode -ne 0) {
    $compileOutput | ForEach-Object { Write-Host $_ }
    throw "Host 级联控制测试编译失败，退出码：$compileExitCode"
}

& $cascadeExecutablePath
if ($LASTEXITCODE -ne 0) {
    throw "Host 级联控制测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/电流速度位置级联逐拍对照测试通过。"

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        -Wno-misleading-indentation `
        "-I$focFakeIncludeDir" "-I$commonDir" "-I$controlDir" `
        "-I$driveDir" "-I$motorDir" "-I$legacyDir" `
        $controlLoopTestSource $controlLoopSource $controlMitSource `
        $cascadeSource $controlPidSource $limitSource $focCoreSource `
        $svmSource $transformSource $legacyControlLoopSource `
        $legacyCascadeSource $legacyPidSource $legacyFocCoreSource `
        $legacyUtilSource `
        -o $controlLoopExecutablePath -lm 2>&1 |
        ForEach-Object { $_.ToString() }
)
$compileExitCode = $LASTEXITCODE

if ($compileExitCode -ne 0) {
    $compileOutput | ForEach-Object { Write-Host $_ }
    throw "Host 可移植控制模式主链测试编译失败，退出码：$compileExitCode"
}

& $controlLoopExecutablePath
if ($LASTEXITCODE -ne 0) {
    throw "Host 可移植控制模式主链测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/电压电流速度位置与 MIT 主链测试通过。"

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        "-I$focFakeIncludeDir" "-I$appDir" "-I$commonDir" "-I$controlDir" `
        "-I$diagnosticIncludeDir" "-I$motorDir" "-I$driveDir" `
        $debugSnapshotTestSource $debugSnapshotSource `
        -o $debugSnapshotExecutablePath 2>&1 |
        ForEach-Object { $_.ToString() }
)
$compileExitCode = $LASTEXITCODE

if ($compileExitCode -ne 0) {
    $compileOutput | ForEach-Object { Write-Host $_ }
    throw "Host 调试快照测试编译失败，退出码：$compileExitCode"
}

& $debugSnapshotExecutablePath
if ($LASTEXITCODE -ne 0) {
    throw "Host 调试快照测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/只读调试快照测试通过。"

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        "-I$focFakeIncludeDir" "-I$appDir" "-I$commonDir" "-I$controlDir" `
        "-I$motorDir" "-I$driveDir" `
        $controlCycleTestSource $controlCycleSource `
        -o $controlCycleExecutablePath 2>&1 |
        ForEach-Object { $_.ToString() }
)
$compileExitCode = $LASTEXITCODE

if ($compileExitCode -ne 0) {
    $compileOutput | ForEach-Object { Write-Host $_ }
    throw "Host 控制周期输入输出测试编译失败，退出码：$compileExitCode"
}

& $controlCycleExecutablePath
if ($LASTEXITCODE -ne 0) {
    throw "Host 控制周期输入输出测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/控制周期显式输入输出测试通过。"

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        "-I$focFakeIncludeDir" "-I$appDir" "-I$bspIncludeDir" "-I$commonDir" `
        "-I$controlDir" "-I$motorDir" "-I$driveDir" `
        $controlReplayTestSource $controlCycleSource $driveSource `
        $driveModeSource $driveCommandSource $driveProtectionSource `
        $drivePwmSource $limitSource `
        -o $controlReplayExecutablePath -lm 2>&1 |
        ForEach-Object { $_.ToString() }
)
$compileExitCode = $LASTEXITCODE

if ($compileExitCode -ne 0) {
    $compileOutput | ForEach-Object { Write-Host $_ }
    throw "Host 控制链离线回放测试编译失败，退出码：$compileExitCode"
}

& $controlReplayExecutablePath
if ($LASTEXITCODE -ne 0) {
    throw "Host 控制链离线回放测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/控制链 STOP-START-RUN-FAULT 离线回放测试通过。"

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        "-I$focFakeIncludeDir" "-I$appDir" "-I$bspIncludeDir" "-I$commonDir" "-I$controlDir" `
        "-I$motorDir" "-I$driveDir" `
        $fastLoopTestSource $fastLoopSource $targetIrqSource $controlCycleSource `
        -o $fastLoopExecutablePath 2>&1 |
        ForEach-Object { $_.ToString() }
)
$compileExitCode = $LASTEXITCODE

if ($compileExitCode -ne 0) {
    $compileOutput | ForEach-Object { Write-Host $_ }
    throw "Host 快速周期编排测试编译失败，退出码：$compileExitCode"
}

& $fastLoopExecutablePath
if ($LASTEXITCODE -ne 0) {
    throw "Host 快速周期编排测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/快速周期编排与对象传递测试通过。"

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        "-I$appDir" "-I$motorDir" "-I$driveDir" `
        $publicHeadersTestSource `
        -o $publicHeadersExecutablePath 2>&1 |
        ForEach-Object { $_.ToString() }
)
$compileExitCode = $LASTEXITCODE

if ($compileExitCode -ne 0) {
    $compileOutput | ForEach-Object { Write-Host $_ }
    throw "Host 对外头文件边界测试编译失败，退出码：$compileExitCode"
}

& $publicHeadersExecutablePath
if ($LASTEXITCODE -ne 0) {
    throw "Host 对外头文件边界测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/App 与 Drive 对外头文件边界测试通过。"

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        "-I$focFakeIncludeDir" "-I$bspIncludeDir" "-I$commonDir" "-I$controlDir" `
        "-I$motorDir" "-I$driveDir" `
        $drivePwmTestSource $drivePwmSource `
        -o $drivePwmExecutablePath 2>&1 |
        ForEach-Object { $_.ToString() }
)
$compileExitCode = $LASTEXITCODE

if ($compileExitCode -ne 0) {
    $compileOutput | ForEach-Object { Write-Host $_ }
    throw "Host Drive PWM 边界测试编译失败，退出码：$compileExitCode"
}

& $drivePwmExecutablePath
if ($LASTEXITCODE -ne 0) {
    throw "Host Drive PWM 边界测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/Drive PWM 与 Fake Target 边界测试通过。"

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        "-I$focFakeIncludeDir" "-I$commonDir" "-I$controlDir" "-I$motorDir" "-I$driveDir" `
        $driveCommandTestSource $driveCommandSource $limitSource `
        -o $driveCommandExecutablePath -lm 2>&1 |
        ForEach-Object { $_.ToString() }
)
$compileExitCode = $LASTEXITCODE

if ($compileExitCode -ne 0) {
    $compileOutput | ForEach-Object { Write-Host $_ }
    throw "Host Drive 命令适配测试编译失败，退出码：$compileExitCode"
}

& $driveCommandExecutablePath
if ($LASTEXITCODE -ne 0) {
    throw "Host Drive 命令适配测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/Drive 命令校验与限幅测试通过。"

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        "-I$focFakeIncludeDir" "-I$commonDir" "-I$controlDir" "-I$motorDir" "-I$driveDir" `
        $driveResetTestSource $driveResetSource $controlPidSource $speedSource `
        -o $driveResetExecutablePath -lm 2>&1 |
        ForEach-Object { $_.ToString() }
)
$compileExitCode = $LASTEXITCODE

if ($compileExitCode -ne 0) {
    $compileOutput | ForEach-Object { Write-Host $_ }
    throw "Host Drive 控制复位测试编译失败，退出码：$compileExitCode"
}

& $driveResetExecutablePath
if ($LASTEXITCODE -ne 0) {
    throw "Host Drive 控制复位测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/Drive 控制运行状态复位测试通过。"

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        "-I$focFakeIncludeDir" "-I$commonDir" "-I$controlDir" "-I$motorDir" "-I$driveDir" `
        $driveModeTestSource $driveModeSource `
        -o $driveModeExecutablePath 2>&1 |
        ForEach-Object { $_.ToString() }
)
$compileExitCode = $LASTEXITCODE

if ($compileExitCode -ne 0) {
    $compileOutput | ForEach-Object { Write-Host $_ }
    throw "Host Drive 模式分派测试编译失败，退出码：$compileExitCode"
}

& $driveModeExecutablePath
if ($LASTEXITCODE -ne 0) {
    throw "Host Drive 模式分派测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/Drive 模式分派测试通过。"

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        "-I$commonDir" "-I$driveDir" `
        $driveProtectionTestSource $driveProtectionSource `
        -o $driveProtectionExecutablePath -lm 2>&1 |
        ForEach-Object { $_.ToString() }
)
$compileExitCode = $LASTEXITCODE

if ($compileExitCode -ne 0) {
    $compileOutput | ForEach-Object { Write-Host $_ }
    throw "Host Drive 保护测试编译失败，退出码：$compileExitCode"
}

& $driveProtectionExecutablePath
if ($LASTEXITCODE -ne 0) {
    throw "Host Drive 保护测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/Drive 保护与故障锁存测试通过。"

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        "-I$focFakeIncludeDir" "-I$commonDir" "-I$controlDir" "-I$motorDir" "-I$driveDir" `
        $driveTestSource $driveSource $driveProtectionSource `
        -o $driveExecutablePath -lm 2>&1 |
        ForEach-Object { $_.ToString() }
)
$compileExitCode = $LASTEXITCODE

if ($compileExitCode -ne 0) {
    $compileOutput | ForEach-Object { Write-Host $_ }
    throw "Host Drive 状态测试编译失败，退出码：$compileExitCode"
}

& $driveExecutablePath
if ($LASTEXITCODE -ne 0) {
    throw "Host Drive 状态测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/Drive 状态测试通过。"

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        "-I$adapterDir" "-I$bspIncludeDir" "-I$commonDir" `
        $boardAdapterTestSource $boardAdapterSource `
        -o $boardAdapterExecutablePath -lm 2>&1 |
        ForEach-Object { $_.ToString() }
)
$compileExitCode = $LASTEXITCODE

if ($compileExitCode -ne 0) {
    $compileOutput | ForEach-Object { Write-Host $_ }
    throw "Host 板卡适配层测试编译失败，退出码：$compileExitCode"
}

& $boardAdapterExecutablePath
if ($LASTEXITCODE -ne 0) {
    throw "Host 板卡适配层测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/板卡相序与ADC换算适配测试通过。"

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        "-I$focFakeIncludeDir" "-I$commonDir" "-I$configDir" "-I$controlDir" `
        "-I$diagnosticIncludeDir" "-I$motorDir" "-I$driveDir" `
        $driveDiagTestSource $driveDiagSource $focCoreSource `
        $svmSource $transformSource `
        -o $driveDiagExecutablePath -lm 2>&1 |
        ForEach-Object { $_.ToString() }
)
$compileExitCode = $LASTEXITCODE

if ($compileExitCode -ne 0) {
    $compileOutput | ForEach-Object { Write-Host $_ }
    throw "Host 诊断适配层测试编译失败，退出码：$compileExitCode"
}

& $driveDiagExecutablePath
if ($LASTEXITCODE -ne 0) {
    throw "Host 诊断适配层测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/诊断采样、FOC 与 PWM 适配测试通过。"

& (Join-Path $PSScriptRoot "run_diagnostic_tests.ps1") -Compiler $Compiler
