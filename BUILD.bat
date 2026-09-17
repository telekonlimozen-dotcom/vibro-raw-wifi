@echo off
setlocal
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0flash.ps1" -Build
endlocal
