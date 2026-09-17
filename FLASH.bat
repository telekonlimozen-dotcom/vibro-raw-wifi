@echo off
setlocal
set PORT=%1
if "%PORT%"=="" (
  echo Usage: FLASH.bat COMxx
  echo Example: FLASH.bat COM27
  exit /b 1
)
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0flash.ps1" -Port %PORT% -Monitor
endlocal
