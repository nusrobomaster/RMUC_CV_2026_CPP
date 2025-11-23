cmake_minimum_required(VERSION 3.5)
project(calibur_pf LANGUAGES CXX)

add_library(calibur_pf STATIC
    rbpf.cu
    rbpf.cuh
)

# Enable CUDA
set_target_properties(calibur_pf PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    POSITION_INDEPENDENT_CODE ON
    LINKER_LANGUAGE CXX
)


target_include_directories(calibur_pf
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/calibur   # for shared headers, if any
)

target_link_libraries(calibur_pf
    PUBLIC
        CUDA::cudart
        Eigen3::Eigen
)
