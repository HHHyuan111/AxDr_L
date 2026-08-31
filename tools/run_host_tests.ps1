param(
    [string]$Compiler = "gcc.exe"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$testDir = Join-Path $repoRoot "tests/host"
$legacyDir = Join-Path $repoRoot "tests/legacy"
$focFakeIncludeDir = Join-Path $testDir "fakes"
$appDir = Join-Path $repoRoot "firmware/AxDr_App/User/app"
$bspIncludeDir = Join-Path $repoRoot "firmware/AxDr_App/User/bsp/inc"
$commonDir = Join-Path $repoRoot "firmware/AxDr_App/User/common"
$controlDir = Join-Path $repoRoot "firmware/AxDr_App/User/control"
$motorDir = Join-Path $repoRoot "firmware/AxDr_App/User/motor"
$driveDir = Join-Path $repoRoot "firmware/AxDr_App/User/drive"
$focTestSource = Join-Path $testDir "test_foc_math.c"
$driveModeTestSource = Join-Path $testDir "test_drive_mode.c"
$driveTestSource = Join-Path $testDir "test_drive_state.c"
$drivePwmTestSource = Join-Path $testDir "test_drive_pwm.c"
$pidTestSource = Join-Path $testDir "test_control_pid.c"
$cascadeTestSource = Join-Path $testDir "test_control_cascade.c"
$debugSnapshotTestSource = Join-Path $testDir "test_debug_snapshot.c"
$controlCycleTestSource = Join-Path $testDir "test_control_cycle.c"
$fastLoopTestSource = Join-Path $testDir "test_fast_loop.c"
$publicHeadersTestSource = Join-Path $testDir "test_public_headers.c"
$controlCycleSource = Join-Path $appDir "control_cycle.c"
$fastLoopSource = Join-Path $appDir "fast_loop.c"
$debugSnapshotSource = Join-Path $appDir "debug_snapshot.c"
$legacyFocSource = Join-Path $legacyDir "legacy_foc.c"
$legacyFocCoreSource = Join-Path $legacyDir "legacy_foc_core.c"
$legacyCascadeSource = Join-Path $legacyDir "legacy_control_cascade.c"
$legacyPidSource = Join-Path $legacyDir "legacy_pid.c"
$legacyUtilSource = Join-Path $legacyDir "legacy_util.c"
$filterSource = Join-Path $controlDir "control_filter.c"
$cascadeSource = Join-Path $controlDir "control_cascade.c"
$focCoreSource = Join-Path $controlDir "foc_core.c"
$limitSource = Join-Path $controlDir "control_limit.c"
$svmSource = Join-Path $controlDir "foc_svm.c"
$transformSource = Join-Path $controlDir "foc_transform.c"
$controlPidSource = Join-Path $controlDir "control_pid.c"
$speedSource = Join-Path $controlDir "control_speed.c"
$utilSource = Join-Path $motorDir "util.c"
$driveSource = Join-Path $driveDir "drive.c"
$driveModeSource = Join-Path $driveDir "drive_mode.c"
$drivePwmSource = Join-Path $driveDir "drive_pwm.c"
$outputDir = Join-Path $repoRoot "firmware/AxDr_App/build/host-tests"
$focExecutablePath = Join-Path $outputDir "test_foc_math.exe"
$driveModeExecutablePath = Join-Path $outputDir "test_drive_mode.exe"
$driveExecutablePath = Join-Path $outputDir "test_drive_state.exe"
$drivePwmExecutablePath = Join-Path $outputDir "test_drive_pwm.exe"
$pidExecutablePath = Join-Path $outputDir "test_control_pid.exe"
$cascadeExecutablePath = Join-Path $outputDir "test_control_cascade.exe"
$debugSnapshotExecutablePath = Join-Path $outputDir "test_debug_snapshot.exe"
$controlCycleExecutablePath = Join-Path $outputDir "test_control_cycle.exe"
$fastLoopExecutablePath = Join-Path $outputDir "test_fast_loop.exe"
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
        "-I$focFakeIncludeDir" "-I$appDir" "-I$commonDir" "-I$controlDir" `
        "-I$motorDir" "-I$driveDir" `
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
        "-I$focFakeIncludeDir" "-I$appDir" "-I$commonDir" "-I$controlDir" `
        "-I$motorDir" "-I$driveDir" `
        $fastLoopTestSource $fastLoopSource $controlCycleSource `
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
        "-I$focFakeIncludeDir" "-I$commonDir" "-I$controlDir" "-I$motorDir" "-I$driveDir" `
        $driveTestSource $driveSource `
        -o $driveExecutablePath 2>&1 |
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
