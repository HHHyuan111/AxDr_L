param()

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$formalDirs = @(
    (Join-Path $repoRoot "firmware/AxDr_App/User/adapter"),
    (Join-Path $repoRoot "firmware/AxDr_App/User/app"),
    (Join-Path $repoRoot "firmware/AxDr_App/User/config"),
    (Join-Path $repoRoot "firmware/AxDr_App/User/control"),
    (Join-Path $repoRoot "firmware/AxDr_App/User/diagnostic"),
    (Join-Path $repoRoot "firmware/AxDr_App/User/drive"),
    (Join-Path $repoRoot "tests/host")
)
$formalFiles = @(
    (Join-Path $repoRoot "firmware/AxDr_App/User/common/compiler.h"),
    (Join-Path $repoRoot "firmware/AxDr_App/User/common/math_const.h"),
    (Join-Path $repoRoot "firmware/AxDr_App/User/common/ret.h"),
    (Join-Path $repoRoot "firmware/AxDr_App/User/common/fault.h"),
    (Join-Path $repoRoot "firmware/AxDr_App/User/common/phase_order.h"),
    (Join-Path $repoRoot "firmware/AxDr_App/User/motor/motor_modes.c")
)

$formalFiles += Get-ChildItem `
    (Join-Path $repoRoot "firmware/AxDr_App/User/bsp") `
    -Recurse -File -Include "target_*.c", "target_*.h" |
    Select-Object -ExpandProperty FullName

foreach ($dir in $formalDirs) {
    $formalFiles += Get-ChildItem $dir -Recurse -File -Include "*.c", "*.h" |
        Select-Object -ExpandProperty FullName
}

$formalFiles = $formalFiles | Sort-Object -Unique
$issues = New-Object "System.Collections.Generic.List[string]"

foreach ($file in $formalFiles) {
    $text = [System.IO.File]::ReadAllText($file)
    $relativePath = $file.Substring($repoRoot.Length + 1)

    if (!$text.EndsWith("`n")) {
        $issues.Add("${relativePath}: 文件末尾缺少换行。")
    }

    $lineNumber = 0
    foreach ($line in ($text -split "`n")) {
        $lineNumber++
        $content = $line.TrimEnd("`r")

        if ($content.Contains("`t")) {
            $issues.Add("${relativePath}:${lineNumber}: 包含制表符。")
        }
        if ($content -match "[ ]+$") {
            $issues.Add("${relativePath}:${lineNumber}: 包含行尾空格。")
        }
        if ($content.Length -gt 120) {
            $issues.Add("${relativePath}:${lineNumber}: 超过 120 字符。")
        }
    }
}

$productionFiles = $formalFiles | Where-Object { $_ -notmatch "[\\/]tests[\\/]host[\\/]" }
foreach ($file in $productionFiles) {
    $text = [System.IO.File]::ReadAllText($file)
    $relativePath = $file.Substring($repoRoot.Length + 1)

    if ($text -match "\b(malloc|calloc|realloc|free)\s*\(") {
        $issues.Add("${relativePath}: 正式控制链禁止动态内存。")
    }
    if ($text -match "\bHAL_Delay\s*\(") {
        $issues.Add("${relativePath}: 正式快速控制链禁止延时等待。")
    }
}

if ($issues.Count -gt 0) {
    $issues | ForEach-Object { Write-Host $_ }
    throw "正式代码规范检查失败，共 $($issues.Count) 项。"
}

Write-Host "正式代码规范检查通过：$($formalFiles.Count) 个 C/H 文件。"
