@echo off
rem  From a fresh Windows PC to a checked install, in one step: a virtual environment (.venv), the pinned packages, the vendor
rem  arm package (a .whl file) and the doctor (a read-only check of this PC and the cell). A plain batch file: Windows runs it
rem  without any script permission, from the Command Prompt or from PowerShell.
rem
rem    setup\install.bat                                   the real cell; the .whl is taken from setup\vendor\ or this folder
rem    setup\install.bat "C:\path\to\bioyondsdk-0.1.1-py3-none-any.whl"
rem    setup\install.bat /sim                              a PC with no cell: the simulator only (no vendor package)
rem
rem  A PC with no internet: put the downloaded packages (.whl files) into setup\vendor\wheels\ and they are used instead.
rem  Then:   cli setup        (in PowerShell:  .\cli setup)

setlocal EnableExtensions
cd /d "%~dp0.."

set "SIM="
set "WHEEL="
:args
if "%~1"=="" goto args_done
if /i "%~1"=="/sim" (set "SIM=1") else if /i "%~1"=="--sim" (set "SIM=1") else if /i "%~1"=="-Sim" (set "SIM=1") else (set "WHEEL=%~1")
shift
goto args
:args_done

echo.
echo == Python
set "PY="
python -c "import sys" >nul 2>nul && set "PY=python"
if not defined PY py -3 -c "import sys" >nul 2>nul && set "PY=py -3"
if not defined PY goto no_python
%PY% -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)"
if errorlevel 1 goto old_python
%PY% -c "import struct, sys; sys.exit(0 if struct.calcsize('P') == 8 else 1)"
if errorlevel 1 goto python_32
%PY% -c "import sys; print('Python %%d.%%d.%%d, 64 bit' %% sys.version_info[:3])"

echo.
echo == Virtual environment (.venv)
if not exist ".venv\Scripts\python.exe" %PY% -m venv .venv
if not exist ".venv\Scripts\python.exe" goto no_venv
set "VPY=.venv\Scripts\python.exe"

echo.
echo == Packages (pinned in setup\requirements.txt)
set "FIND="
if exist "setup\vendor\wheels\*.whl" set "FIND=--no-index --find-links setup\vendor\wheels"
if defined FIND echo using the packages in setup\vendor\wheels (no internet needed)
if not defined FIND "%VPY%" -m pip install --upgrade pip --quiet
set "REQ=setup\requirements.txt"
if defined SIM (
    findstr /v /b /i /c:"pythonnet" "setup\requirements.txt" > "%TEMP%\polar-requirements-sim.txt"
    set "REQ=%TEMP%\polar-requirements-sim.txt"
)
"%VPY%" -m pip install %FIND% -r "%REQ%"
if errorlevel 1 goto no_packages

if defined SIM goto doctor
echo.
echo == Vendor arm package (bioyondsdk .whl)
if not defined WHEEL for %%F in ("setup\vendor\bioyondsdk-*.whl" "bioyondsdk-*.whl") do set "WHEEL=%%~fF"
if not defined WHEEL goto no_wheel_given
if not exist "%WHEEL%" goto wheel_missing
"%VPY%" -m pip install %FIND% "%WHEEL%"
if errorlevel 1 goto wheel_failed

:doctor
echo.
echo == Doctor (reads only, moves nothing)
if defined SIM ("%VPY%" setup\doctor.py --sim) else ("%VPY%" setup\doctor.py)
set "CODE=%ERRORLEVEL%"
echo.
if defined SIM (echo Next:  cli setup --sim) else (echo Next:  cli setup      ^(in PowerShell:  .\cli setup^))
exit /b %CODE%

:no_wheel_given
echo.
echo No vendor .whl was given and none is in setup\vendor\ : the real arm needs it. The doctor below says so.
echo Get bioyondsdk-0.1.1-py3-none-any.whl from Cephla or Bioyond, then:  setup\install.bat "C:\path\to\that.whl"
goto doctor

:no_python
echo FAILED: no usable Python found. Install Python 3.10 or newer, 64 bit, from python.org and tick "Add python.exe to PATH".
echo         If Windows opens the Microsoft Store when you type python: Settings, Apps, Advanced app settings, App execution aliases, switch python.exe off.
exit /b 1
:old_python
echo FAILED: this Python is too old: 3.10 or newer is needed (python.org).
exit /b 1
:python_32
echo FAILED: this Python is 32 bit. The arm's vendor package needs the 64-bit Python: python.org, "Windows installer (64-bit)".
exit /b 1
:no_venv
echo FAILED: could not create .venv in %CD%. Is the folder writable?
exit /b 1
:no_packages
echo FAILED: pip could not install the packages. With internet: check the network and run this again.
echo         Without internet: put the .whl files of setup\requirements.txt into setup\vendor\wheels\ and run this again.
exit /b 1
:wheel_missing
echo FAILED: the file "%WHEEL%" does not exist.
exit /b 1
:wheel_failed
echo FAILED: pip could not install "%WHEEL%".
exit /b 1
