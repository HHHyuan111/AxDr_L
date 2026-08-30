param(
    [string]$FirmwareDir = (Join-Path (Split-Path -Parent $PSScriptRoot) "firmware/AxDr_App")
)

$ErrorActionPreference = "Stop"

# 这里只比较项目自行维护的 User C 源文件；CubeMX/HAL 源文件由各自构建系统管理。
$cmakePath = Join-Path $FirmwareDir "CMakeLists.txt"
$keilPath = Join-Path $FirmwareDir "MDK-ARM/AxDr.uvprojx"

if (-not (Test-Path -LiteralPath $cmakePath)) {
    throw "未找到 CMake 工程文件：$cmakePath"
}

if (-not (Test-Path -LiteralPath $keilPath)) {
    throw "未找到 Keil 工程文件：$keilPath"
}

$cmakeText = Get-Content -LiteralPath $cmakePath -Raw
$sourceBlock = [regex]::Match(
    $cmakeText,
    'set\s*\(AXDR_USER_SOURCES(?<body>.*?)\)',
    [System.Text.RegularExpressions.RegexOptions]::Singleline
)

if (-not $sourceBlock.Success) {
    throw "CMakeLists.txt 中未找到 AXDR_USER_SOURCES。"
}

$cmakeSources = [regex]::Matches(
    $sourceBlock.Groups['body'].Value,
    '(?m)^\s*(?<path>User/[A-Za-z0-9_./-]+\.c)\s*$'
) | ForEach-Object {
    $_.Groups['path'].Value
} | Sort-Object -Unique

$keilProject = New-Object System.Xml.XmlDocument
$keilProject.Load($keilPath)
$keilSources = $keilProject.SelectNodes('//FilePath') | ForEach-Object {
    $path = $_.'#text'.Replace('\', '/')

    while ($path.StartsWith('../')) {
        $path = $path.Substring(3)
    }

    if ($path -match '^User/.+\.c$') {
        $path
    }
} | Sort-Object -Unique

$cmakeOnly = Compare-Object $cmakeSources $keilSources |
    Where-Object SideIndicator -eq '<=' |
    ForEach-Object InputObject
$keilOnly = Compare-Object $cmakeSources $keilSources |
    Where-Object SideIndicator -eq '=>' |
    ForEach-Object InputObject

Write-Host "CMake 用户源文件：$($cmakeSources.Count) 个"
Write-Host "Keil 用户源文件：$($keilSources.Count) 个"

if (($cmakeOnly.Count -eq 0) -and ($keilOnly.Count -eq 0)) {
    Write-Host "检查通过：CMake 与 Keil 的 User C 源文件清单一致。"
    exit 0
}

if ($cmakeOnly.Count -gt 0) {
    Write-Host "仅在 CMake 中启用："
    $cmakeOnly | ForEach-Object { Write-Host "  $_" }
}

if ($keilOnly.Count -gt 0) {
    Write-Host "仅在 Keil 中启用："
    $keilOnly | ForEach-Object { Write-Host "  $_" }
}

exit 1
