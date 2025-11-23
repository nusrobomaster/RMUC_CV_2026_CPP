cmake_minimum_required(VERSION 3.10)

set(CUDAToolkit_ROOT "/usr/local/cuda" CACHE PATH "CUDA Toolkit root")

project(yolo11_pose LANGUAGES CXX CUDA)

add_definitions(-DAPI_EXPORTS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# -------------------- Eigen --------------------
include_directories("/usr/include/eigen3")

# -------------------- CUDA (modern CUDAToolkit style) ----
find_package(CUDAToolkit REQUIRED)
include_directories(${CUDAToolkit_INCLUDE_DIRS})
link_directories(${CUDAToolkit_LIBRARY_DIR})

# -------------------- TensorRT --------------------
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    message("Embed_platform on")
    include_directories(/usr/include/aarch64-linux-gnu)
    link_directories(/usr/lib/aarch64-linux-gnu)
else()
    message("Embed_platform off")
    include_directories(/usr/include/x86_64-linux-gnu)
    link_directories(/usr/lib/x86_64-linux-gnu)
endif()

# -------------------- OpenCV --------------------
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# -------------------- YOLO Include --------------------
include_directories(${PROJECT_SOURCE_DIR}/include)

# -------------------- YOLO Inference Library --------------------
file(GLOB_RECURSE YOLO_SRC
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/src/*.cu
)

add_library(yolo_infer SHARED ${YOLO_SRC})
set_target_properties(yolo_infer PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

target_link_libraries(yolo_infer
    cudart
    nvinfer
    nvonnxparser
    ${OpenCV_LIBS}
)

# -------------------- Optional standalone main --------------------
# add_executable(main ${PROJECT_SOURCE_DIR}/main.cpp)
# target_link_libraries(main yolo_infer)
