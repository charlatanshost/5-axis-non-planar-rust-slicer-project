@echo off
REM Double-click launcher for the MultiAxis Slicer GUI.
REM Delegates to run-gui.ps1; any arguments are passed straight through,
REM so "run-gui.bat -Fast" and "run-gui.bat -Rebuild" both work.

setlocal
set "SCRIPT_DIR=%~dp0"

REM Prefer PowerShell 7 (pwsh) and fall back to Windows PowerShell.
where pwsh >nul 2>&1
if %ERRORLEVEL%==0 (
    pwsh -NoProfile -ExecutionPolicy Bypass -File "%SCRIPT_DIR%run-gui.ps1" %*
) else (
    powershell -NoProfile -ExecutionPolicy Bypass -File "%SCRIPT_DIR%run-gui.ps1" %*
)

REM Keep the window open on failure so a double-clicked launcher does not
REM vanish before the error can be read.
if errorlevel 1 (
    echo.
    echo Launcher exited with an error.
    pause
)
endlocal
