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
    $candidates = @(
        (Join-Path $env:USERPROFILE ".espressif\python_env\idf5.5_py3.12_env\Scripts\python.exe"),
        (Join-Path $env:USERPROFILE ".espressif\python_env\idf5.4_py3.11_env\Scripts\python.exe"),
        "$env:LOCALAPPDATA\Programs\Python\Python312\python.exe",
        "$env:LOCALAPPDATA\Programs\Python\Python311\python.exe"
    )
    foreach ($py in $candidates) {
        if ($py -and (Test-Path -LiteralPath $py)) {
            return @{ Exe = $py; Args = @("-m", "esptool") }
        }
    }
    $cmd = Get-Command python -ErrorAction SilentlyContinue
    if ($cmd) {
        return @{ Exe = $cmd.Source; Args = @("-m", "esptool") }
    }
    throw "python/esptool not found. Install ESP-IDF or: pip install esptool"
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
        throw "No binaries in release\. Run BUILD.bat first."
    }
    New-Item -ItemType Directory -Force -Path $Release | Out-Null
    Copy-Item $bBin $bin -Force
    Copy-Item $bBoot $boot -Force
    Copy-Item $bPart $part -Force
    Set-Content (Join-Path $Release "VERSION.txt") "0.4.1-rawwifi" -Encoding Ascii
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
    Set-Content (Join-Path $Release "VERSION.txt") "0.4.1-rawwifi" -Encoding Ascii
    Write-Host "OK -> $Release" -ForegroundColor Green
    if (-not $Port) { exit 0 }
}

Ensure-Release

if (-not $Port) {
    Write-Host "Usage: FLASH.bat COMxx" -ForegroundColor Yellow
    Write-Host "Bins in: $Release"
    exit 1
}

if ($Port -notmatch '^COM\d+$') {
    throw "Bad port '$Port' (expected COMxx, e.g. COM27)"
}

$et = Find-Esptool
$bin = Join-Path $Release "vibro_raw_wifi.bin"
$boot = Join-Path $Release "bootloader.bin"
$part = Join-Path $Release "partition-table.bin"

Write-Host "Using $($et.Exe)" -ForegroundColor DarkGray
Write-Host "Flashing ESP32-C6 on $Port ..." -ForegroundColor Cyan
$flashArgs = $et.Args + @(
    "--chip", "esp32c6", "-p", $Port, "-b", "$Baud",
    "--before", "default_reset", "--after", "hard_reset",
    "write_flash", "--flash_mode", "dio", "--flash_freq", "80m", "--flash_size", "detect",
    "0x0", $boot, "0x8000", $part, "0x10000", $bin
)
& $et.Exe @flashArgs
if ($LASTEXITCODE -ne 0) { throw "flash failed (exit $LASTEXITCODE)" }
Write-Host "Flash OK" -ForegroundColor Green

if ($Monitor) {
    $export = Find-IdfExport
    cmd /c "call `"$export`" && cd /d `"$Root`" && idf.py -p $Port monitor"
}
