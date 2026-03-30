@echo off
setlocal
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0git_upload.ps1"
set ERR=%ERRORLEVEL%
if not "%ERR%"=="0" pause
exit /b %ERR%
