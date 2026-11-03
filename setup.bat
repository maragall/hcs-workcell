@echo off
rem The one-click setup (cli.py setup) with the two live cameras beside it. Extra arguments pass through, e.g.  setup.bat --from "grips.toml"
cd /d "%~dp0"
set PYTHONIOENCODING=utf-8
start "" "C:\Users\ASUS\AppData\Local\Programs\Python\Python312\pythonw.exe" gui.py
"C:\Users\ASUS\AppData\Local\Programs\Python\Python312\python.exe" -X utf8 cli.py setup %*
