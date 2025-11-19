#include <cstring>
#include "types.hpp"
#include "workers.hpp"

CameraWorker::CameraWorker(void* cam_handle,
                SharedLatest &shared,
                std::atomic<bool> &stop_flag)
    : cam_(cam_handle), shared_(shared), stop_(stop_flag) {}

void CameraWorker::operator()() {
    // MV_CC_StartGrabbing(cam_);

    while (!stop_.load(std::memory_order_relaxed)) {
        CameraFrame frame;
        grab_frame_stub(frame); // TODO: real camera

        auto ptr = std::make_shared<CameraFrame>(std::move(frame));
        std::atomic_store(&shared_.camera, ptr);
        shared_.camera_ver.fetch_add(1, std::memory_order_relaxed);
    }

    // MV_CC_StopGrabbing(cam_);
}


void CameraWorker::grab_frame_stub(CameraFrame &frame) {
    frame.timestamp = Clock::now();
    frame.width  = 640;
    frame.height = 480;
    frame.raw_data.resize(frame.width * frame.height * 3);
    // TODO: fill from real SDK
}

