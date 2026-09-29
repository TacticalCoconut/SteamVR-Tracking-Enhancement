@echo off
rem Removes the driver's registration from SteamVR. Files are left where they are.
setlocal
set ROOT=%~dp0
set PKG=%ROOT%dist\svrenhance
tasklist /fi "imagename eq vrserver.exe" 2>nul | find /i "vrserver.exe" >nul && (
  echo SteamVR is running. Close it, then run this again.
  exit /b 1
)
call :find_vrpathreg || exit /b 1
"%VRPATHREG%" removedriver "%PKG%" || (echo vrpathreg failed. & exit /b 1)
echo Uninstalled. SteamVR no longer loads the driver.
echo Settings under "driver_svrenhance" in steamvr.vrsettings and the logs in "%LOCALAPPDATA%\SVREnhance" were left in place.
exit /b 0

:find_vrpathreg
set VRPATHREG=%ProgramFiles(x86)%\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe
if exist "%VRPATHREG%" exit /b 0
set RUNTIME=
for /f "usebackq delims=" %%i in (`powershell -NoProfile -Command "try { (Get-Content (Join-Path $env:LOCALAPPDATA 'openvr\openvrpaths.vrpath') -Raw | ConvertFrom-Json).runtime[0] } catch { }"`) do set RUNTIME=%%i
set VRPATHREG=%RUNTIME%\bin\win64\vrpathreg.exe
if defined RUNTIME if exist "%VRPATHREG%" exit /b 0
echo SteamVR was not found.
exit /b 1
