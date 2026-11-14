@echo off
rem  Command Prompt only: after this, `python` is the installed environment in this window (python demo.py, python cli.py ...).
rem  Not needed for `cli ...`, and it has no effect in PowerShell: there use  .\cli setup  and  .\.venv\Scripts\python demo.py
if not exist "%~dp0..\.venv\Scripts\activate.bat" (
    echo Not installed yet: run setup\install.bat first.
    exit /b 1
)
call "%~dp0..\.venv\Scripts\activate.bat"
