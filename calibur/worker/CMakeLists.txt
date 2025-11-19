cmake_minimum_required(VERSION 3.5)

add_library(calibur_worker STATIC
    PFWorker.cpp
)

target_include_directories(calibur_worker PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(calibur_worker
    PUBLIC
        calibur_pf
)

