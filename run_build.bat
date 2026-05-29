@echo off
call "C:\Xilinx\Vitis\2023.2\settings64.bat"
call "C:\Xilinx\Vitis\2023.2\tps\win64\lopper-1.1.0\env\Scripts\activate.bat"
set PATH=C:\Xilinx\Vitis\2023.2\tps\win64\cmake-3.24.2\bin;%PATH%
cmake --version
python "C:\Xilinx\Vitis\2023.2\data\embeddedsw\scripts\pyesw\build_app.py" ^
    -s "C:\Users\kimse\capstone\antidrone\vitis_workspace\antidrone_app\src" ^
    -b "C:\Users\kimse\capstone\antidrone\vitis_workspace\antidrone_app\build"
echo BUILD_RESULT=%ERRORLEVEL%
