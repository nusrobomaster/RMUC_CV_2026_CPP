# calibur/worker/CMakeLists.txt

set(WORKER_SOURCES
    camera_worker.cpp
    detection_worker.cpp
    imu_worker.cpp
    pf_worker.cpp
    prediction_worker.cpp
    usb_worker.cpp
    yolo_worker.cpp
    helper.hpp
    thread_pool.hpp
    types.hpp
    workers.hpp
)

add_library(calibur_worker_core STATIC
    ${WORKER_SOURCES}
)

target_include_directories(calibur_worker_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/calibur
        ${CMAKE_SOURCE_DIR}/calibur/pose/include
        ${CMAKE_SOURCE_DIR}/calibur/pf
)

target_link_libraries(calibur_worker_core
    PUBLIC
        calibur_deps     # OpenCV + Eigen + yaml-cpp + CUDA + platform
        yolo_pose     # from calibur/pose
        calibur_pf       # from calibur/pf
)
