param(
    [string]$Compiler = "gcc.exe"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$testDir = Join-Path $repoRoot "tests/host"
$fakeIncludeDir = Join-Path $testDir "fakes"
$motorDir = Join-Path $repoRoot "firmware/AxDr_App/User/motor"
$testSource = Join-Path $testDir "test_foc_math.c"
$focSource = Join-Path $motorDir "foc_calc.c"
$utilSource = Join-Path $motorDir "util.c"
$outputDir = Join-Path $repoRoot "firmware/AxDr_App/build/host-tests"
$executablePath = Join-Path $outputDir "test_foc_math.exe"

$compilerCommand = Get-Command $Compiler -ErrorAction SilentlyContinue
if ($null -eq $compilerCommand) {
    throw "未找到电脑端 C 编译器。请把 gcc.exe 加入 PATH，或通过 -Compiler 指定路径。"
}

New-Item -ItemType Directory -Path $outputDir -Force | Out-Null

$compileOutput = @(
    & $compilerCommand.Source -std=c11 -Wall -Wextra -Wpedantic -Werror `
        "-I$fakeIncludeDir" "-I$motorDir" `
        $testSource $focSource $utilSource `
        -o $executablePath -lm 2>&1 |
        ForEach-Object { $_.ToString() }
)
$compileExitCode = $LASTEXITCODE

if ($compileExitCode -ne 0) {
    $compileOutput | ForEach-Object { Write-Host $_ }
    throw "Host FOC 数学测试编译失败，退出码：$compileExitCode"
}

& $executablePath
if ($LASTEXITCODE -ne 0) {
    throw "Host FOC 数学测试运行失败，退出码：$LASTEXITCODE"
}

Write-Host "Host C11/FOC 数学测试通过。"
