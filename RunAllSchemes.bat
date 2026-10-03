@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0RunAllSchemes.ps1" %*
exit /b %ERRORLEVEL%
