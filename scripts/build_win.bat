@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" > nul 2>&1

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

set "BUILD_DIR=C:\Users\kimse\capstone\antidrone\cpp\build_win"
set "CMAKE=C:\Program Files\CMake\bin\cmake.exe"

rmdir /s /q "%BUILD_DIR%" 2>nul
mkdir "%BUILD_DIR%"

echo [1/2] CMake configure...
"%CMAKE%" -G "NMake Makefiles" ^
    -DCMAKE_BUILD_TYPE=Release ^
    "-DOpenCV_DIR=C:/opencv/build/x64/vc16/lib" ^
    "-DONNXRUNTIME_DIR=C:/Users/kimse/capstone/onnxruntime-gpu/onnxruntime-win-x64-gpu-1.20.1" ^
    -S "C:/Users/kimse/capstone/antidrone/cpp" ^
    -B "%BUILD_DIR%"

if %ERRORLEVEL% NEQ 0 (
    echo CMake configure 실패!
    pause
    exit /b 1
)

echo [2/2] Build...
"%CMAKE%" --build "%BUILD_DIR%" --config Release -- -j4

if %ERRORLEVEL% NEQ 0 (
    echo Build 실패!
    pause
    exit /b 1
)

echo.
echo ============================
echo  빌드 완료!
echo  실행 파일 위치: %BUILD_DIR%
echo ============================
