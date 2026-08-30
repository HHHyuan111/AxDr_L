param(
    [string]$CMakeCommand = "cmake.exe"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$firmwareDir = Join-Path $repoRoot "firmware/AxDr_App"
$warningBaseline = [ordered]@{
    Debug = 104
    Release = 104
}

$cmake = Get-Command $CMakeCommand -ErrorAction SilentlyContinue
if ($null -eq $cmake) {
    throw "未找到 CMake。请先把交叉编译工具目录加入 PATH，或通过 -CMakeCommand 指定路径。"
}

Push-Location $firmwareDir
try {
    foreach ($entry in $warningBaseline.GetEnumerator()) {
        $preset = $entry.Key
        $warningLimit = $entry.Value
        $buildOutput = @(
            & $cmake.Source --build --preset $preset --clean-first 2>&1 |
                ForEach-Object { $_.ToString() }
        )
        $buildExitCode = $LASTEXITCODE
        $warningLines = @($buildOutput | Where-Object { $_ -match 'warning:' })

        if ($buildExitCode -ne 0) {
            $buildOutput | ForEach-Object { Write-Host $_ }
            throw "$preset 构建失败，退出码：$buildExitCode"
        }

        if ($warningLines.Count -gt $warningLimit) {
            $warningLines | ForEach-Object { Write-Host $_ }
            throw "$preset 警告增加：当前 $($warningLines.Count) 条，基线 $warningLimit 条。"
        }

        Write-Host "$preset 构建通过：0 个错误，$($warningLines.Count) 个警告，基线不超过 $warningLimit 个。"
    }
}
finally {
    Pop-Location
}
