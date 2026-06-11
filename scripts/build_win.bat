@echo off
setlocal

set "VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
if exist "%VCVARS%" goto HaveVS
echo [FAIL] Visual Studio Build Tools vcvars64.bat not found:
echo        %VCVARS%
exit /b 1

:HaveVS
call "%VCVARS%" > nul 2>&1

set "WINSDK_VER=10.0.26100.0"
set "WINSDK_BIN=C:\Program Files (x86)\Windows Kits\10\bin\%WINSDK_VER%\x64"
set "WINSDK_LIB_UM=C:\Program Files (x86)\Windows Kits\10\Lib\%WINSDK_VER%\um\x64"
set "WINSDK_LIB_UCRT=C:\Program Files (x86)\Windows Kits\10\Lib\%WINSDK_VER%\ucrt\x64"
set "WINSDK_INCLUDE_UM=C:\Program Files (x86)\Windows Kits\10\Include\%WINSDK_VER%\um"
set "WINSDK_INCLUDE_UCRT=C:\Program Files (x86)\Windows Kits\10\Include\%WINSDK_VER%\ucrt"
set "WINSDK_INCLUDE_SHARED=C:\Program Files (x86)\Windows Kits\10\Include\%WINSDK_VER%\shared"

set "PATH=%PATH%;%WINSDK_BIN%"
set "LIB=%LIB%;%WINSDK_LIB_UM%;%WINSDK_LIB_UCRT%"
set "INCLUDE=%INCLUDE%;%WINSDK_INCLUDE_UM%;%WINSDK_INCLUDE_UCRT%;%WINSDK_INCLUDE_SHARED%"

set "ROOT=%~dp0.."
for %%I in ("%ROOT%") do set "ROOT=%%~fI"
set "CPP_DIR=%ROOT%\antidrone\cpp"
set "BUILD_DIR=%CPP_DIR%\build_win"
set "CMAKE=C:\Program Files\CMake\bin\cmake.exe"
set "ONNXRUNTIME_DIR=%ROOT%\onnxruntime-gpu\onnxruntime-win-x64-gpu-1.20.1"

if exist "%CMAKE%" goto HaveCMake
echo [FAIL] CMake not found:
echo        %CMAKE%
exit /b 1

:HaveCMake

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

echo [1/2] CMake configure...
"%CMAKE%" -G "NMake Makefiles" ^
    -DCMAKE_BUILD_TYPE=Release ^
    "-DOpenCV_DIR=C:/opencv/build/x64/vc16/lib" ^
    "-DONNXRUNTIME_DIR=%ONNXRUNTIME_DIR:\=/%" ^
    -S "%CPP_DIR%" ^
    -B "%BUILD_DIR%"

if %ERRORLEVEL% NEQ 0 (
    echo CMake configure failed.
    exit /b 1
)

echo [2/2] Build...
"%CMAKE%" --build "%BUILD_DIR%" --target ptcamera_tracker --config Release

if %ERRORLEVEL% NEQ 0 (
    echo Build failed.
    exit /b 1
)

echo [2/2] Build benchmark (추적 정량화 도구)...
"%CMAKE%" --build "%BUILD_DIR%" --target benchmark --config Release

if %ERRORLEVEL% NEQ 0 (
    echo Benchmark build failed.
    exit /b 1
)

echo.
echo ============================
echo  Build complete.
echo  Output: %BUILD_DIR%
echo ============================
