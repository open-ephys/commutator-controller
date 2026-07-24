@echo off
pushd %~dp0
powershell -ExecutionPolicy Bypass -File ./flash-utility.ps1 %*
set "RC=%ERRORLEVEL%"
popd
exit /b %RC%