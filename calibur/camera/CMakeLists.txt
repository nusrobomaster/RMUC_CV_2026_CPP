add_executable(test_camera
    camera.cpp
)

# Header search paths
target_include_directories(test_camera
    PRIVATE
        ${CMAKE_SOURCE_DIR}              # for "calibur/camera/..."
        ${CMAKE_CURRENT_SOURCE_DIR}      # for "MvCameraControl.h"
        /opt/MVS/include                 # HIK SDK headers (check if headers are here)
)

# Find the HIK/MindVision camera library
find_library(MVCAMERA_LIB
    NAMES MvCameraControl
    PATHS /opt/MVS/lib/aarch64
    NO_DEFAULT_PATH
)

if(NOT MVCAMERA_LIB)
    message(FATAL_ERROR "Could not find MvCameraControl library in /opt/MVS/lib/aarch64")
endif()

# Libraries to link
target_link_libraries(test_camera
    PRIVATE
        ${OpenCV_LIBS}
        ${MVCAMERA_LIB}     # full path ensures linker finds it
)
