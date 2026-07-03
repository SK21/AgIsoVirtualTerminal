@echo off
setlocal
set "VSCMD_START_DIR=%CD%"
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat" x64
if errorlevel 1 (
    echo Failed to set up Visual Studio environment
    exit /b 1
)
cd /d "F:\Documents\GitHub\AgIsoVirtualTerminal\build"
echo Starting build...
nmake 2>&1
echo Build exit code: %errorlevel%
