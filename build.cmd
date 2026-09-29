@echo off
rem Builds everything with MSVC (x64, static CRT):
rem   dist\svrenhance\                       the SteamVR driver package
rem   dist\tools\svrenhance_fix.exe          Quick Fix panel (and its .vrmanifest)
rem   dist\tools\svrenhance_roomsetup.exe    Room Setup
rem Usage: build.cmd [driver^|tools]          (no argument: both)
setlocal
set ROOT=%~dp0
set WHAT=%1
set VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe
if not exist "%VSWHERE%" (
  echo Visual Studio was not found. Install "Build Tools for Visual Studio" with the "Desktop development with C++" workload.
  exit /b 1
)
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find VC\Auxiliary\Build\vcvars64.bat`) do set VCVARS=%%i
if not defined VCVARS (
  echo The MSVC x64 compiler was not found. Add the "Desktop development with C++" workload in the Visual Studio Installer.
  exit /b 1
)
call "%VCVARS%" >nul 2>nul || exit /b 1

set CFLAGS=/nologo /std:c++17 /O2 /EHsc /MT /W4 /WX /permissive- /D_CRT_SECURE_NO_WARNINGS /external:W0 /external:I "%ROOT%third_party\openvr"

if /i "%WHAT%"=="tools" goto tools

rem ---------------------------------------------------------------- driver
set OUT=%ROOT%dist\svrenhance
set OBJ=%ROOT%build\driver
if not exist "%OUT%\bin\win64" mkdir "%OUT%\bin\win64"
if not exist "%OBJ%" mkdir "%OBJ%"
xcopy /y /e /q "%ROOT%driver\package\svrenhance\*" "%OUT%\" >nul || exit /b 1

cl %CFLAGS% /DNOMINMAX /DWIN32_LEAN_AND_MEAN /Fo"%OBJ%\\" ^
   "%ROOT%driver\src\driver_main.cpp" "%ROOT%driver\src\filter.cpp" "%ROOT%driver\src\hook.cpp" "%ROOT%driver\src\telemetry.cpp" ^
   /LD /Fe"%OBJ%\driver_svrenhance.dll" /link /NOLOGO /IMPLIB:"%OBJ%\driver_svrenhance.lib" || exit /b 1

rem A running vrserver keeps the installed DLL open; it cannot be overwritten, but it can be moved
rem aside, and the new file is picked up when SteamVR next starts.
set DST=%OUT%\bin\win64\driver_svrenhance.dll
if exist "%DST%" (
  if not exist "%ROOT%build\_to_delete" mkdir "%ROOT%build\_to_delete"
  move /y "%DST%" "%ROOT%build\_to_delete\driver_svrenhance.%RANDOM%%RANDOM%.dll" >nul || (echo Could not move the old DLL aside & exit /b 1)
)
copy /y "%OBJ%\driver_svrenhance.dll" "%DST%" >nul || exit /b 1
echo Built %DST%
tasklist /fi "imagename eq vrserver.exe" 2>nul | find /i "vrserver.exe" >nul && echo SteamVR is running: the new driver loads on its next start.

if /i "%WHAT%"=="driver" goto done

rem ---------------------------------------------------------------- tools
:tools
set OUT=%ROOT%dist\tools
set OBJ=%ROOT%build\tools
if not exist "%OUT%" mkdir "%OUT%"
if not exist "%OBJ%\fix" mkdir "%OBJ%\fix"
if not exist "%OBJ%\roomsetup" mkdir "%OBJ%\roomsetup"

cl %CFLAGS% /I "%ROOT%tools\common" /Fo"%OBJ%\fix\\" "%ROOT%tools\fix\main.cpp" ^
   /Fe"%OUT%\svrenhance_fix.exe" /link /NOLOGO user32.lib gdi32.lib || exit /b 1
copy /y "%ROOT%tools\fix\svrenhance_fix.vrmanifest" "%OUT%\" >nul || exit /b 1
echo Built %OUT%\svrenhance_fix.exe

cl %CFLAGS% /I "%ROOT%tools\common" /Fo"%OBJ%\roomsetup\\" "%ROOT%tools\roomsetup\main.cpp" ^
   /Fe"%OUT%\svrenhance_roomsetup.exe" /link /NOLOGO user32.lib gdi32.lib || exit /b 1
echo Built %OUT%\svrenhance_roomsetup.exe

:done
exit /b 0
