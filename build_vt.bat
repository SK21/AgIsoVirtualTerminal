@echo off
setlocal

REM Set up Visual Studio environment
set "VSCMD_START_DIR=%CD%"
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat" x64
if errorlevel 1 (
    echo Failed to set up Visual Studio environment
    exit /b 1
)

cd /d "%~dp0"

REM Clean and configure
if exist build rmdir /s /q build
mkdir build
cd build

echo Configuring CMake...
"C:\Program Files\CMake\bin\cmake.exe" -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release .. > build_log.txt 2>&1
if errorlevel 1 (
    echo CMake configuration failed!
    type build_log.txt
    exit /b 1
)
type build_log.txt

echo Building...
"C:\Program Files\CMake\bin\cmake.exe" --build . --config Release > build_log2.txt 2>&1
if errorlevel 1 (
    echo Build failed!
    type build_log2.txt
    exit /b 1
)
type build_log2.txt

echo Build succeeded!
echo Done!
