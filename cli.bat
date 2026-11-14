@echo off
rem  python cli.py with the installed environment, no activation needed:   cli setup      (in PowerShell:  .\cli setup)
if not exist "%~dp0.venv\Scripts\python.exe" (
    echo Not installed yet: run setup\install.bat first.
    exit /b 1
)
"%~dp0.venv\Scripts\python.exe" "%~dp0cli.py" %*
