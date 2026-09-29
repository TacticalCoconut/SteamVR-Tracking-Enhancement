@echo off
rem Builds and runs every offline test. No SteamVR and no headset needed.
setlocal
set ROOT=%~dp0
set VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe
if not exist "%VSWHERE%" (
  echo Visual Studio was not found. Install "Build Tools for Visual Studio" with the "Desktop development with C++" workload.
  exit /b 1
)
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find VC\Auxiliary\Build\vcvars64.bat`) do set VCVARS=%%i
if not defined VCVARS (
  echo The MSVC x64 compiler was not found.
  exit /b 1
)
call "%VCVARS%" >nul 2>nul || exit /b 1

set CFLAGS=/nologo /std:c++17 /O2 /EHsc /MT /W4 /WX /permissive- /D_CRT_SECURE_NO_WARNINGS /external:W0 /external:I "%ROOT%third_party\openvr"
set OBJ=%ROOT%build\test
for %%d in (driver geom fix) do if not exist "%OBJ%\%%d" mkdir "%OBJ%\%%d"
set FAIL=0

echo === driver: pose pipeline, station monitor, world correction ===
cl %CFLAGS% /DNOMINMAX /DWIN32_LEAN_AND_MEAN /DSVR_TEST_CLOCK /Fo"%OBJ%\driver\\" ^
   "%ROOT%tests\filter_test.cpp" "%ROOT%driver\src\filter.cpp" "%ROOT%driver\src\hook.cpp" "%ROOT%driver\src\telemetry.cpp" ^
   /Fe"%OBJ%\driver\filter_test.exe" /link /NOLOGO >"%OBJ%\driver\compile.log" 2>&1 || (type "%OBJ%\driver\compile.log" & exit /b 1)
"%OBJ%\driver\filter_test.exe" || set FAIL=1

echo.
echo === room setup: geometry ===
cl %CFLAGS% /I "%ROOT%tools\common" /Fo"%OBJ%\geom\\" "%ROOT%tools\roomsetup\geom_test.cpp" ^
   /Fe"%OBJ%\geom\geom_test.exe" /link /NOLOGO >"%OBJ%\geom\compile.log" 2>&1 || (type "%OBJ%\geom\compile.log" & exit /b 1)
"%OBJ%\geom\geom_test.exe" || set FAIL=1

echo.
echo === quick fix: decisions ===
cl %CFLAGS% /I "%ROOT%tools\common" /Fo"%OBJ%\fix\\" "%ROOT%tools\fix\fixmath_test.cpp" ^
   /Fe"%OBJ%\fix\fixmath_test.exe" /link /NOLOGO >"%OBJ%\fix\compile.log" 2>&1 || (type "%OBJ%\fix\compile.log" & exit /b 1)
"%OBJ%\fix\fixmath_test.exe" || set FAIL=1

echo.
if "%FAIL%"=="0" (echo ALL TESTS PASSED) else (echo SOME TESTS FAILED)
exit /b %FAIL%
