@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat" x64
if errorlevel 1 (
    echo VCVARS_FAILED
    exit /b 1
)
echo VCVARS_OK
cd /d "F:\Documents\GitHub\AgIsoVirtualTerminal\build"
nmake 2>&1
if errorlevel 1 (
    echo BUILD_FAILED
    exit /b 1
)
echo BUILD_OK
