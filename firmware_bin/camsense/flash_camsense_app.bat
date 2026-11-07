@echo off
setlocal
if "%~1"=="" (
  echo Usage: flash_camsense_app.bat COM5 [baud]
  echo This writes only the app partition at 0x10000 and keeps WiFi/NVS.
  exit /b 1
)
if "%~2"=="" (
  powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0flash_camsense_app.ps1" -Port %~1
) else (
  powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0flash_camsense_app.ps1" -Port %~1 -Baud %~2
)
pause
