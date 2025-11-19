cmake_minimum_required(VERSION 3.5)

add_library(calibur_pf STATIC
    rbpf_posyaw.cu
)

# Enable CUDA
set_target_properties(calibur_pf PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    POSITION_INDEPENDENT_CODE ON
)

# Export the include directory (pf/)
target_include_directories(calibur_pf PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
