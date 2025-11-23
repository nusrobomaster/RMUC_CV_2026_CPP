cmake_minimum_required(VERSION 3.5)
project(calibur LANGUAGES CXX)



# Make CUDAToolkit_ROOT actually used, and avoid CUDA_ARCH warnings
if (POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()
if (POLICY CMP0104)
    cmake_policy(SET CMP0104 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -rdynamic -O3 -g -Wall")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

#=================================== change cuda dir to the desired dir ==============================================
# set(CUDAToolkit_ROOT "/usr/local/cuda" CACHE PATH "CUDA Toolkit root")
# find_package(CUDAToolkit REQUIRED)
find_package(CUDAToolkit REQUIRED)
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

# =========================================================
#  Platform detection: x86_64 Linux vs Jetson (aarch64)
# =========================================================
set(CALIBUR_PLATFORM "auto" CACHE STRING "calibur target platform (auto|x86|jetson)")

if (CALIBUR_PLATFORM STREQUAL "auto")
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        set(CALIBUR_PLATFORM "jetson")
    elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        set(CALIBUR_PLATFORM "x86")
    else()
        message(FATAL_ERROR "Unknown arch '${CMAKE_SYSTEM_PROCESSOR}', please set CALIBUR_PLATFORM manually.")
    endif()
endif()

message(STATUS "CALIBUR_PLATFORM = ${CALIBUR_PLATFORM}")

# For Jetson: which L4T (36 = JP6.2, 38 = JP7.x)?
set(CALIBUR_L4T "" CACHE STRING "L4T major version (36 for JetPack 6.x, 38 for JetPack 7.x)")

add_library(calibur_platform INTERFACE)

if (CALIBUR_PLATFORM STREQUAL "jetson")
    target_compile_definitions(calibur_platform INTERFACE CALIBUR_PLATFORM_JETSON=1)

    # Try auto-detect L4T if user didn’t set it
    if (NOT CALIBUR_L4T AND EXISTS "/etc/nv_tegra_release")
        file(READ "/etc/nv_tegra_release" NV_TEGRA_REL)
        if (NV_TEGRA_REL MATCHES "R36")
            set(CALIBUR_L4T "36" CACHE STRING "detected L4T version" FORCE)
        elseif (NV_TEGRA_REL MATCHES "R38")
            set(CALIBUR_L4T "38" CACHE STRING "detected L4T version" FORCE)
        endif()
    endif()

    message(STATUS "CALIBUR_L4T = ${CALIBUR_L4T}")

    if (CALIBUR_L4T STREQUAL "36")
        # JetPack 6.2 (L4T 36.x)
        target_compile_definitions(calibur_platform INTERFACE CALIBUR_L4T36=1)
        # Typical system include/lib paths
        target_include_directories(calibur_platform INTERFACE
            /usr/include/aarch64-linux-gnu
        )
        target_link_directories(calibur_platform INTERFACE
            /usr/lib/aarch64-linux-gnu
        )
        # CUDA archs for AGX Orin / Orin NX etc (tune if needed)
        set(CMAKE_CUDA_ARCHITECTURES 87 CACHE STRING "" FORCE)

    elseif (CALIBUR_L4T STREQUAL "38")
        # JetPack 7.x (L4T 38.x)
        target_compile_definitions(calibur_platform INTERFACE CALIBUR_L4T38=1)
        target_include_directories(calibur_platform INTERFACE
            /usr/include/aarch64-linux-gnu
        )
        target_link_directories(calibur_platform INTERFACE
            /usr/lib/aarch64-linux-gnu
        )
        # For Thor / newer, adjust if you know exact SM
        set(CMAKE_CUDA_ARCHITECTURES 87 CACHE STRING "" FORCE)  # or 90 etc

    else()
        message(WARNING "Unknown L4T version; set CALIBUR_L4T to 36 or 38 explicitly.")
    endif()

elseif (CALIBUR_PLATFORM STREQUAL "x86")
    target_compile_definitions(calibur_platform INTERFACE CALIBUR_PLATFORM_X86=1)

    target_include_directories(calibur_platform INTERFACE
        /usr/include/x86_64-linux-gnu
    )
    target_link_directories(calibur_platform INTERFACE
        /usr/lib/x86_64-linux-gnu
    )
else()
    message(FATAL_ERROR "Unsupported CALIBUR_PLATFORM='${CALIBUR_PLATFORM}'")
endif()


# ----------------- Dependencies -----------------

find_package(OpenCV REQUIRED core imgproc imgcodecs highgui calib3d)
find_package(Eigen3 REQUIRED)

list(APPEND CMAKE_PREFIX_PATH
     "${CMAKE_SOURCE_DIR}/apps/yaml-cpp/lib/cmake")
find_package(yaml-cpp REQUIRED)

# Optionally expose common deps via another interface target
add_library(calibur_deps INTERFACE)
target_link_libraries(calibur_deps INTERFACE
    calibur_platform
    yaml-cpp::yaml-cpp
    Eigen3::Eigen
    ${OpenCV_LIBS}
    CUDA::cudart 
)

# Any target in subdirs can now:  target_link_libraries(my_target PRIVATE calibur_deps)

# ----------------- Subdirectories -----------------
add_subdirectory(calibur/pf)
add_subdirectory(calibur/pose)
add_subdirectory(calibur/worker)
# add_subdirectory(tests)

add_executable(calibur_worker
    main.cpp      # or wherever you put your main()
)

target_link_libraries(calibur_worker
    PRIVATE
        calibur_worker_core
        calibur_deps
)



