@echo off
rem The setup in a plain console (the desktop shortcut opens gui.py instead). Extra arguments pass through, e.g.  setup.bat --from "grips.toml"
cd /d "%~dp0"
set PYTHONIOENCODING=utf-8
python -X utf8 cli.py setup %*
