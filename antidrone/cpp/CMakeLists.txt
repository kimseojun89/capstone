cmake_minimum_required(VERSION 3.16)

project(ptcamera_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)

# ---------------------------------------------------------------------------
# ONNX Runtime GPU (CUDA 12.1, RTX 3050)
# ---------------------------------------------------------------------------
set(ONNXRUNTIME_DIR
    "C:/Users/kimse/capstone/onnxruntime-gpu/onnxruntime-win-x64-gpu-1.20.1"
    CACHE PATH "ONNX Runtime GPU 설치 경로")

add_library(onnxruntime SHARED IMPORTED GLOBAL)
set_target_properties(onnxruntime PROPERTIES
    IMPORTED_IMPLIB             "${ONNXRUNTIME_DIR}/lib/onnxruntime.lib"
    IMPORTED_LOCATION           "${ONNXRUNTIME_DIR}/lib/onnxruntime.dll"
    INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_DIR}/include"
)

# ---------------------------------------------------------------------------
# Core library
# ---------------------------------------------------------------------------
add_library(ptcamera_core
    src/control.cpp
    src/detector.cpp
    src/overlay.cpp
    src/pipeline.cpp
    src/serial_port.cpp
    src/settings.cpp
    src/tracking.cpp
)

target_include_directories(ptcamera_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_definitions(ptcamera_core
    PUBLIC
        PTCAMERA_PROJECT_ROOT="${CMAKE_CURRENT_LIST_DIR}/.."
)

target_link_libraries(ptcamera_core
    PUBLIC
        ${OpenCV_LIBS}
        onnxruntime
)

# ---------------------------------------------------------------------------
# 실행 파일별 빌드 후 ONNX Runtime DLL 복사
# ---------------------------------------------------------------------------
function(add_ptcamera_app target source)
    add_executable(${target} ${source})
    target_link_libraries(${target} PRIVATE ptcamera_core)

    # 실행 파일 옆에 ONNX Runtime DLL 복사 (CUDA 프로바이더 포함)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${ONNXRUNTIME_DIR}/lib/onnxruntime.dll"
            "${ONNXRUNTIME_DIR}/lib/onnxruntime_providers_cuda.dll"
            "${ONNXRUNTIME_DIR}/lib/onnxruntime_providers_shared.dll"
            "$<TARGET_FILE_DIR:${target}>"
        COMMENT "Copying ONNX Runtime DLLs for ${target}"
    )
endfunction()

add_ptcamera_app(camera_smoke_test    apps/camera_smoke_test.cpp)
add_ptcamera_app(detector_viewer      apps/detector_viewer.cpp)
add_ptcamera_app(tracking_viewer      apps/tracking_viewer.cpp)
add_ptcamera_app(ptcamera_tracker     apps/ptcamera_tracker.cpp)
if(WIN32)
    target_link_libraries(ptcamera_tracker PRIVATE ws2_32)
endif()
