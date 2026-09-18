@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title vibro-raw-wifi flash

set "PORT=%~1"
if not "%PORT%"=="" goto :run

echo.
echo === vibro-raw-wifi 0.4.3 flash ===
echo.
echo Available COM ports:
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -match 'COM\d+' } | ForEach-Object { $_.Name } | Sort-Object"
echo.
set /p PORT=Enter port (e.g. COM27): 
if "%PORT%"=="" (
  echo No port entered.
  pause
  exit /b 1
)

:run
echo.
echo Flashing on %PORT% ...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0flash.ps1" -Port "%PORT%"
set "ERR=%ERRORLEVEL%"
echo.
if not "%ERR%"=="0" (
  echo FLASH FAILED code=%ERR%
) else (
  echo FLASH OK
)
pause
exit /b %ERR%

