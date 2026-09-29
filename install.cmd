@echo off
rem Registers the built driver with SteamVR, using SteamVR's own vrpathreg. Undo with uninstall.cmd.
setlocal
set ROOT=%~dp0
set PKG=%ROOT%dist\svrenhance
if not exist "%PKG%\bin\win64\driver_svrenhance.dll" (
  echo The driver has not been built yet. Run build.cmd first, or unpack a release so that dist\svrenhance exists.
  exit /b 1
)
tasklist /fi "imagename eq vrserver.exe" 2>nul | find /i "vrserver.exe" >nul && (
  echo SteamVR is running. Close it, then run this again.
  exit /b 1
)
call :find_vrpathreg || exit /b 1
"%VRPATHREG%" adddriver "%PKG%" || (echo vrpathreg failed. & exit /b 1)
echo Installed. Start SteamVR; "%LOCALAPPDATA%\SVREnhance\driver.log" should then say "pose hook verified".
exit /b 0

:find_vrpathreg
set VRPATHREG=%ProgramFiles(x86)%\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe
if exist "%VRPATHREG%" exit /b 0
set RUNTIME=
for /f "usebackq delims=" %%i in (`powershell -NoProfile -Command "try { (Get-Content (Join-Path $env:LOCALAPPDATA 'openvr\openvrpaths.vrpath') -Raw | ConvertFrom-Json).runtime[0] } catch { }"`) do set RUNTIME=%%i
set VRPATHREG=%RUNTIME%\bin\win64\vrpathreg.exe
if defined RUNTIME if exist "%VRPATHREG%" exit /b 0
echo SteamVR was not found. Install SteamVR and start it once, then run this again.
exit /b 1
