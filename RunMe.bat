@echo off
setlocal
title Experiment Launcher

rem ============================================================
rem If command-line arguments are supplied, keep the old behavior.
rem Example:
rem   RunAllSchemes.bat -Scheme1
rem   RunAllSchemes.bat -Resume
rem ============================================================

if not "%~1"=="" (
    powershell -NoProfile -ExecutionPolicy Bypass ^
        -File "%~dp0RunAllSchemes.ps1" %*
    exit /b %ERRORLEVEL%
)

:MENU
cls

echo ==========================================
echo          Experiment Launcher
echo ==========================================
echo.
echo   [1] Run all schemes
echo   [2] Run Scheme1 only
echo   [3] Run Scheme2 only
echo.
echo   [4] Resume unfinished experiments
echo   [5] Retry failed experiments
echo.
echo   [6] Clear generated experiment files
echo.
echo   [0] Exit
echo.
echo ==========================================

choice /C 1234560 /N /M "Select mode: "

if errorlevel 7 goto EXIT
if errorlevel 6 goto CLEAR
if errorlevel 5 goto RETRY
if errorlevel 4 goto RESUME
if errorlevel 3 goto SCHEME2
if errorlevel 2 goto SCHEME1
if errorlevel 1 goto ALL

:ALL
cls
echo Running all schemes...
echo.
powershell -NoProfile -ExecutionPolicy Bypass ^
    -File "%~dp0RunAllSchemes.ps1"
goto FINISH

:SCHEME1
cls
echo Running Scheme1...
echo.
powershell -NoProfile -ExecutionPolicy Bypass ^
    -File "%~dp0RunAllSchemes.ps1" -Scheme1
goto FINISH

:SCHEME2
cls
echo Running Scheme2...
echo.
powershell -NoProfile -ExecutionPolicy Bypass ^
    -File "%~dp0RunAllSchemes.ps1" -Scheme2
goto FINISH

:RESUME
cls
echo Resuming unfinished experiments...
echo.
powershell -NoProfile -ExecutionPolicy Bypass ^
    -File "%~dp0RunAllSchemes.ps1" -Resume
goto FINISH

:RETRY
cls
echo Retrying failed experiments...
echo.
powershell -NoProfile -ExecutionPolicy Bypass ^
    -File "%~dp0RunAllSchemes.ps1" -RetryFailed
goto FINISH

:CLEAR
cls
echo ==========================================
echo WARNING
echo ==========================================
echo.
echo This will remove:
echo.
echo   results.csv
echo   runtime/
echo   Results/
echo   generated temporary files
echo   generated PNG figures
echo   Scheme1 Nfe_config.txt
echo.
echo Source code and task data will NOT be removed.
echo.

choice /C YN /N /M "Continue? [Y/N]: "

if errorlevel 2 goto MENU
if errorlevel 1 goto DOCLEAR

:DOCLEAR
powershell -NoProfile -ExecutionPolicy Bypass ^
    -File "%~dp0RunAllSchemes.ps1" -Clear

goto FINISH

:FINISH
set EXITCODE=%ERRORLEVEL%

echo.
echo ==========================================
if "%EXITCODE%"=="0" (
    echo Operation finished successfully.
) else (
    echo Operation finished with exit code %EXITCODE%.
)
echo ==========================================
echo.
pause
goto MENU

:EXIT
endlocal
exit /b 0
