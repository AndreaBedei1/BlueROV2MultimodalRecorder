@echo off
setlocal
for %%I in ("%~dp0..") do set "BLUEROV_RECORDER_ROOT=%%~fI"
cd /d "%BLUEROV_RECORDER_ROOT%"
set "PYTHONPATH=%BLUEROV_RECORDER_ROOT%\src;%PYTHONPATH%"

if defined VIRTUAL_ENV (
    set "BLUEROV_RECORDER_PYTHON=python"
) else if exist "%BLUEROV_RECORDER_ROOT%\.venv\Scripts\python.exe" (
    set "BLUEROV_RECORDER_PYTHON=%BLUEROV_RECORDER_ROOT%\.venv\Scripts\python.exe"
) else (
    where py >nul 2>nul
    if not errorlevel 1 (
        set "BLUEROV_RECORDER_PYTHON=py"
    ) else (
        set "BLUEROV_RECORDER_PYTHON=python"
    )
)

rem Safe default: starts the desktop Camera/Ping1D/ROVL recorder only.
%BLUEROV_RECORDER_PYTHON% -m bluerov_recorder.app %*
set "BLUEROV_RECORDER_EXIT=%ERRORLEVEL%"
endlocal & exit /b %BLUEROV_RECORDER_EXIT%
