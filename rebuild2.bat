@echo off
echo REBUILD2 STARTING
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat" x64
if errorlevel 1 (
    echo VCVARS_FAIL
    exit /b 1
)
echo VCVARS_OK
cd /d "F:\Documents\GitHub\AgIsoVirtualTerminal\build"
echo Starting nmake...
nmake 2>&1
if errorlevel 1 (
    echo BUILD_FAIL
    exit /b 1
)
echo BUILD_OK
