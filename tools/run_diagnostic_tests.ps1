param(
    [string]$Compiler = "gcc.exe"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$diagRoot = Join-Path $repoRoot "firmware/AxDr_App/User/diagnostic"
$diagCore = Join-Path $diagRoot "core"
$diagInclude = Join-Path $diagRoot "include"
$controlInclude = Join-Path $repoRoot "firmware/AxDr_App/User/control"
$testRoot = Join-Path $repoRoot "tests/host"
$outputDir = Join-Path $repoRoot "firmware/AxDr_App/build/host-tests"
$controlTest = Join-Path $testRoot "test_mc_control_and_sweep.c"
$identTest = Join-Path $testRoot "test_mc_ident_algorithms.c"
$runtimeTest = Join-Path $testRoot "test_diag_runtime.c"
$controlExe = Join-Path $outputDir "test_mc_control_and_sweep.exe"
$identExe = Join-Path $outputDir "test_mc_ident_algorithms.exe"
$runtimeExe = Join-Path $outputDir "test_diag_runtime.exe"

$compilerCommand = Get-Command $Compiler -ErrorAction SilentlyContinue
if ($null -eq $compilerCommand)
{
    throw "未找到电脑端 C 编译器。请把 gcc.exe 加入 PATH，或通过 -Compiler 指定路径。"
}

New-Item -ItemType Directory -Path $outputDir -Force | Out-Null

$controlSources = @(
    $controlTest,
    (Join-Path $diagCore "mc_common.c"),
    (Join-Path $diagCore "mc_bias_bandwidth.c"),
    (Join-Path $diagCore "mc_current_pi.c"),
    (Join-Path $diagCore "mc_current_sweep.c"),
    (Join-Path $diagCore "mc_deadtime_comp.c"),
    (Join-Path $diagCore "mc_deadtime_test.c"),
    (Join-Path $diagCore "mc_decoupling.c"),
    (Join-Path $diagCore "mc_diag_manager.c"),
    (Join-Path $diagCore "mc_encoder_alignment.c"),
    (Join-Path $diagCore "mc_flux_observer.c")
)

& $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
    "-I$diagInclude" $controlSources -o $controlExe -lm
if ($LASTEXITCODE -ne 0)
{
    throw "诊断控制与扫频测试编译失败，退出码：$LASTEXITCODE"
}

& $controlExe
if ($LASTEXITCODE -ne 0)
{
    throw "诊断控制与扫频测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/诊断控制与扫频算法测试通过。"

$identSources = @(
    $identTest,
    (Join-Path $diagCore "mc_common.c"),
    (Join-Path $diagCore "mc_rs_ident.c"),
    (Join-Path $diagCore "mc_biased_l_ident.c"),
    (Join-Path $diagCore "mc_pole_pair_ident.c")
)

& $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
    "-I$diagInclude" $identSources -o $identExe -lm
if ($LASTEXITCODE -ne 0)
{
    throw "参数辨识测试编译失败，退出码：$LASTEXITCODE"
}

& $identExe
if ($LASTEXITCODE -ne 0)
{
    throw "参数辨识测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/Rs、L 和极对数辨识算法测试通过。"

$runtimeSources = @(
    $runtimeTest,
    (Join-Path $diagCore "diag_runtime.c"),
    (Join-Path $diagCore "mc_common.c"),
    (Join-Path $diagCore "mc_bias_bandwidth.c"),
    (Join-Path $diagCore "mc_biased_l_ident.c"),
    (Join-Path $diagCore "mc_current_pi.c"),
    (Join-Path $diagCore "mc_current_sweep.c"),
    (Join-Path $diagCore "mc_deadtime_comp.c"),
    (Join-Path $diagCore "mc_deadtime_test.c"),
    (Join-Path $diagCore "mc_decoupling.c"),
    (Join-Path $diagCore "mc_diag_manager.c"),
    (Join-Path $diagCore "mc_encoder_alignment.c"),
    (Join-Path $diagCore "mc_flux_observer.c"),
    (Join-Path $diagCore "mc_pole_pair_ident.c"),
    (Join-Path $diagCore "mc_rs_ident.c")
)

& $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
    "-I$diagInclude" "-I$controlInclude" $runtimeSources -o $runtimeExe -lm
if ($LASTEXITCODE -ne 0)
{
    throw "诊断任务状态机测试编译失败，退出码：$LASTEXITCODE"
}

& $runtimeExe
if ($LASTEXITCODE -ne 0)
{
    throw "诊断任务状态机测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/诊断任务状态机与PI单位转换测试通过。"
