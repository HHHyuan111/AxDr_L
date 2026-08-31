param()

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$powerShellPath = (Get-Process -Id $PID).Path

function Invoke-CheckScript {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Name
    )

    $scriptPath = Join-Path $PSScriptRoot $Name
    & $powerShellPath -NoProfile -File $scriptPath
    if ($LASTEXITCODE -ne 0) {
        throw "检查脚本失败：$Name，退出码：$LASTEXITCODE"
    }
}

Invoke-CheckScript "check_formal_code.ps1"
Invoke-CheckScript "run_host_tests.ps1"
Invoke-CheckScript "check_user_source_parity.ps1"
Invoke-CheckScript "check_firmware_build.ps1"

Push-Location $repoRoot
try {
    $diffCheck = @(
        & git diff --check 2>&1 |
        ForEach-Object { $_.ToString() }
    )
    $diffExitCode = $LASTEXITCODE
}
finally {
    Pop-Location
}

if ($diffExitCode -ne 0) {
    $diffCheck | ForEach-Object { Write-Host $_ }
    throw "Git 空白字符检查失败。"
}

Write-Host "发布候选离线验收全部通过。"
