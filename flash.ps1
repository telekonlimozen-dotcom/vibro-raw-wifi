#Requires -Version 5.1
param(
    [Parameter(Position = 0)][string]$Port = "",
    [int]$Baud = 460800,
    [switch]$Monitor,
    [switch]$Build
)

$ErrorActionPreference = "Stop"
$Root = $PSScriptRoot
$Release = Join-Path $Root "release"
$BuildDir = Join-Path $Root "build"

function Find-IdfExport {
    foreach ($p in @(
        $env:IDF_PATH,
        "C:\Espressif\frameworks\esp-idf-v5.5",
        "C:\Espressif\frameworks\esp-idf"
    )) {
        if (-not $p) { continue }
        $ex = Join-Path $p "export.bat"
        if (Test-Path -LiteralPath $ex) { return $ex }
    }
    throw "ESP-IDF export.bat not found"
}

function Find-Esptool {
    $py = Join-Path $env:USERPROFILE ".espressif\python_env\idf5.5_py3.12_env\Scripts\python.exe"
    if (Test-Path $py) { return @($py, "-m", "esptool") }
    $py2 = "$env:LOCALAPPDATA\Programs\Python\Python312\python.exe"
    if (Test-Path $py2) { return @($py2, "-m", "esptool") }
    return @("python", "-m", "esptool")
}

function Ensure-Release {
    $bin = Join-Path $Release "vibro_raw_wifi.bin"
    $boot = Join-Path $Release "bootloader.bin"
    $part = Join-Path $Release "partition-table.bin"
    if ((Test-Path $bin) -and (Test-Path $boot) -and (Test-Path $part)) { return }

    $bBin = Join-Path $BuildDir "vibro_raw_wifi.bin"
    $bBoot = Join-Path $BuildDir "bootloader\bootloader.bin"
    $bPart = Join-Path $BuildDir "partition_table\partition-table.bin"
    if (-not ((Test-Path $bBin) -and (Test-Path $bBoot) -and (Test-Path $bPart))) {
        throw "No release/build binaries. Run: .\BUILD.bat   then .\FLASH.bat COMxx"
    }
    New-Item -ItemType Directory -Force -Path $Release | Out-Null
    Copy-Item $bBin (Join-Path $Release "vibro_raw_wifi.bin") -Force
    Copy-Item $bBoot (Join-Path $Release "bootloader.bin") -Force
    Copy-Item $bPart (Join-Path $Release "partition-table.bin") -Force
    Set-Content (Join-Path $Release "VERSION.txt") "0.4.0-rawwifi" -Encoding utf8
}

if ($Build) {
    $export = Find-IdfExport
    Write-Host "Building via $export" -ForegroundColor Cyan
    $cmd = "call `"$export`" && cd /d `"$Root`" && if exist sdkconfig del /q sdkconfig && idf.py set-target esp32c6 && idf.py build"
    cmd /c $cmd
    if ($LASTEXITCODE -ne 0) { throw "build failed" }
    New-Item -ItemType Directory -Force -Path $Release | Out-Null
    Copy-Item (Join-Path $BuildDir "vibro_raw_wifi.bin") (Join-Path $Release "vibro_raw_wifi.bin") -Force
    Copy-Item (Join-Path $BuildDir "bootloader\bootloader.bin") (Join-Path $Release "bootloader.bin") -Force
    Copy-Item (Join-Path $BuildDir "partition_table\partition-table.bin") (Join-Path $Release "partition-table.bin") -Force
    Set-Content (Join-Path $Release "VERSION.txt") "0.4.0-rawwifi" -Encoding utf8
    Write-Host "OK → $Release" -ForegroundColor Green
    if (-not $Port) { exit 0 }
}

Ensure-Release

if (-not $Port) {
    Write-Host "Usage: .\FLASH.bat COMxx   or   .\flash.ps1 -Port COMxx [-Monitor]" -ForegroundColor Yellow
    Write-Host "Bins in: $Release"
    exit 1
}

$et = Find-Esptool
$bin = Join-Path $Release "vibro_raw_wifi.bin"
$boot = Join-Path $Release "bootloader.bin"
$part = Join-Path $Release "partition-table.bin"

Write-Host "Flashing ESP32-C6 on $Port ..." -ForegroundColor Cyan
& $et[0] $et[1..($et.Length-1)] --chip esp32c6 -p $Port -b $Baud --before default_reset --after hard_reset write_flash `
    --flash_mode dio --flash_freq 80m --flash_size detect `
    0x0 $boot `
    0x8000 $part `
    0x10000 $bin

if ($LASTEXITCODE -ne 0) { throw "flash failed" }
Write-Host "Flash OK" -ForegroundColor Green

if ($Monitor) {
    $export = Find-IdfExport
    cmd /c "call `"$export`" && cd /d `"$Root`" && idf.py -p $Port monitor"
}
