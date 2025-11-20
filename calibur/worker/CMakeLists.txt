cmake_minimum_required(VERSION 3.5)
project(calibur_worker LANGUAGES CXX)

# Collect all .cpp files in this directory (your 5 worker cpp files)
file(GLOB WORKER_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library(calibur_worker STATIC
    ${WORKER_SOURCES}
)

target_include_directories(calibur_worker PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Particle filter library from calibur/pf
target_link_libraries(calibur_worker
    PUBLIC
        calibur_pf
)

# Eigen
find_package(Eigen3 REQUIRED)

target_link_libraries(calibur_worker
    PUBLIC
        Eigen3::Eigen
)
