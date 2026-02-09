@echo off
setlocal

set ROOT=%~dp0
cd /d "%ROOT%"

git status >nul 2>&1
if errorlevel 1 (
  echo [ERROR] Git is not available or this folder is not a git repository.
  echo Initialize git and set remote first.
  exit /b 1
)

echo Updating repository...
git pull --rebase

endlocal
