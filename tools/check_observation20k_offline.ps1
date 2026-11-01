param(
    [string]$Compiler = 'gcc.exe',
    [string]$OutputDirectory = 'artifacts/expert-20k-20261028/offline-verified'
)
$ErrorActionPreference = 'Stop'
$Compiler = (Get-Command $Compiler -ErrorAction Stop).Source
$env:PATH = (Split-Path -Parent $Compiler) + ';' + $env:PATH
$repo = Split-Path -Parent $PSScriptRoot
$out = [IO.Path]::GetFullPath((Join-Path $repo $OutputDirectory))
if (Test-Path -LiteralPath $out) { throw '检查输出目录必须是新目录，保留旧失败证据。' }
New-Item -ItemType Directory -Path $out | Out-Null
$userDir = Join-Path $repo 'firmware/AxDr_App/User'
$testDir = Join-Path $repo 'tests/host'
$service = Join-Path $userDir 'service'
$include = @('-I' + (Join-Path $testDir 'fakes'))
foreach ($folder in @('service','common','config','control','algorithm','motor','drive','adapter','bsp/inc','diagnostic/include')) {
    $include += '-I' + (Join-Path $userDir $folder)
}
$core = @('mc_common','mc_bias_bandwidth','mc_current_pi','mc_current_sweep','mc_deadtime_comp','mc_deadtime_test','mc_decoupling','mc_diag_manager','mc_encoder_alignment','mc_flux_observer','mc_param_ident','mc_pole_pair_ident','diag_runtime') |
    ForEach-Object { Join-Path $userDir "diagnostic/core/$_.c" }
$targets = @(
    @{Name='test_service_observation'; Sources=@("$service/service_observation.c","$service/axdr_command_core.c")},
    @{Name='test_service_usb_tx'; Sources=@("$service/service_usb_tx.c")},
    @{Name='test_usb_cdc_bridge'; Sources=@("$service/service_usb_tx.c")},
    @{Name='test_service_scope'; Sources=@("$service/service_usb.c","$service/service_scope.c","$service/service_observation.c","$service/axdr_command_core.c","$service/axdr_safety_runtime.c","$service/service_telemetry.c") + $core}
)
foreach ($target in $targets) {
    $exe = Join-Path $out ($target.Name + '.exe')
    $compileArgs = @('-std=c11','-Wall','-Wextra','-Wpedantic','-Werror') + $include + @("$testDir/$($target.Name).c") + $target.Sources + @('-o',$exe,'-lm')
    $compileArgs | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $out ($target.Name + '-compile.json'))
    & $Compiler @compileArgs 2>&1 | Tee-Object -FilePath (Join-Path $out ($target.Name + '-compile.txt'))
    if ($LASTEXITCODE -ne 0) { throw "$($target.Name) 编译失败" }
    $runArgs = @()
    if ($target.Name -eq 'test_service_observation') { $runArgs = @($out) }
    & $exe @runArgs 2>&1 | Tee-Object -FilePath (Join-Path $out ($target.Name + '.txt'))
    if ($LASTEXITCODE -ne 0) { throw "$($target.Name) 检查失败" }
}
Write-Output '定向 C 检查完成；未访问设备，不证明 USB 实机吞吐。'
