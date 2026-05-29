# PTCamera C++ Tracker

This directory contains the desktop C++ side of the pan/tilt camera tracker.
The Arduino firmware remains in the repository root PlatformIO project.

## Dependencies

Install C++ development packages for:

- OpenCV, including HighGUI, VideoIO, DNN, and preferably tracking modules
- OpenVINO Runtime C++ development package
- CMake and a C++17 compiler

If CMake cannot find either package, pass the package config directory:

```bash
cmake -S cpp -B cpp/build \
  -DOpenCV_DIR=/path/to/opencv/lib/cmake/opencv4 \
  -DOpenVINO_DIR=/path/to/openvino/runtime/cmake
```

## Build

```bash
cmake -S cpp -B cpp/build
cmake --build cpp/build -j
```

## Step-by-step programs

```bash
./cpp/build/camera_smoke_test --camera 0
./cpp/build/serial_stepper_test --port /dev/ttyACM0 --pan 200 --tilt 0
./cpp/build/detector_viewer --camera 0 --device CPU
./cpp/build/tracking_viewer --camera 0 --device CPU
./cpp/build/ptcamera_tracker --camera 0 --serial-port /dev/ttyACM0
```

`ptcamera_tracker` starts with the motor disabled unless `--enable-motor` is
passed. Press Space to toggle motor output and ESC or `q` to exit.
