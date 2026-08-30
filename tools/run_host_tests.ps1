param(
    [string]$Compiler = "gcc.exe"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$testDir = Join-Path $repoRoot "tests/host"
$focFakeIncludeDir = Join-Path $testDir "fakes"
$appDir = Join-Path $repoRoot "firmware/AxDr_App/User/app"
$controlDir = Join-Path $repoRoot "firmware/AxDr_App/User/control"
$motorDir = Join-Path $repoRoot "firmware/AxDr_App/User/motor"
$driveDir = Join-Path $repoRoot "firmware/AxDr_App/User/drive"
$focTestSource = Join-Path $testDir "test_foc_math.c"
$driveTestSource = Join-Path $testDir "test_drive_state.c"
$pidTestSource = Join-Path $testDir "test_control_pid.c"
$debugSnapshotTestSource = Join-Path $testDir "test_debug_snapshot.c"
$debugSnapshotSource = Join-Path $appDir "debug_snapshot.c"
$focSource = Join-Path $motorDir "foc_calc.c"
$legacyPidSource = Join-Path $motorDir "pid.c"
$filterSource = Join-Path $controlDir "control_filter.c"
$limitSource = Join-Path $controlDir "control_limit.c"
$svmSource = Join-Path $controlDir "foc_svm.c"
$transformSource = Join-Path $controlDir "foc_transform.c"
$controlPidSource = Join-Path $controlDir "control_pid.c"
$utilSource = Join-Path $motorDir "util.c"
$driveSource = Join-Path $driveDir "drive.c"
$outputDir = Join-Path $repoRoot "firmware/AxDr_App/build/host-tests"
$focExecutablePath = Join-Path $outputDir "test_foc_math.exe"
$driveExecutablePath = Join-Path $outputDir "test_drive_state.exe"
$pidExecutablePath = Join-Path $outputDir "test_control_pid.exe"
$debugSnapshotExecutablePath = Join-Path $outputDir "test_debug_snapshot.exe"

$compilerCommand = Get-Command $Compiler -ErrorAction SilentlyContinue
if ($null -eq $compilerCommand) {
    throw "未找到电脑端 C 编译器。请把 gcc.exe 加入 PATH，或通过 -Compiler 指定路径。"
}

New-Item -ItemType Directory -Path $outputDir -Force | Out-Null

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        "-I$focFakeIncludeDir" "-I$controlDir" "-I$motorDir" `
        $focTestSource $focSource $filterSource $limitSource $svmSource $transformSource $utilSource `
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
        "-I$focFakeIncludeDir" "-I$controlDir" "-I$motorDir" `
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
        "-I$focFakeIncludeDir" "-I$appDir" "-I$controlDir" "-I$motorDir" `
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
        "-I$focFakeIncludeDir" "-I$controlDir" "-I$motorDir" "-I$driveDir" `
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
