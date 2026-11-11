@echo off
cd /d "%~dp0"
set OUT=build-codex\last_speed_run.txt
echo === interpretes === > %OUT%
where python py >> %OUT% 2>&1
py -0p >> %OUT% 2>&1
for /f "delims=" %%P in ('py -0p 2^>nul') do echo %%P >nul
set PYEXE=
for %%C in ("py -3" "python") do (
  if not defined PYEXE (
    %%~C -c "import serial" >nul 2>&1 && set "PYEXE=%%~C"
  )
)
if not defined PYEXE (
  for /d %%D in ("%LOCALAPPDATA%\Programs\Python\Python*") do (
    if not defined PYEXE ( "%%D\python.exe" -c "import serial" >nul 2>&1 && set "PYEXE="%%D\python.exe"" )
  )
)
if not defined PYEXE (
  for /d %%D in ("%USERPROFILE%\.espressif\python_env\*" "C:\Espressif\python_env\*" "%USERPROFILE%\.espressif\tools\python\*\venv") do (
    if not defined PYEXE ( if exist "%%D\Scripts\python.exe" ( "%%D\Scripts\python.exe" -c "import serial" >nul 2>&1 && set "PYEXE="%%D\Scripts\python.exe"" ) )
  )
)
echo PYEXE=%PYEXE% >> %OUT%
if not defined PYEXE (
  py -3 -m pip install --user pyserial >> %OUT% 2>&1
  py -3 -c "import serial" >nul 2>&1 && set "PYEXE=py -3"
)
echo PYEXE=%PYEXE% >> %OUT%
if not defined PYEXE ( echo NO_PYSERIAL >> %OUT% & goto end )
%PYEXE% tests\run_cascade_smoke.py --run --port COM17 --stage speed >> %OUT% 2>&1
echo EXIT=%errorlevel% >> %OUT%
:end
type %OUT%
timeout /t 10
