#include <cmath>
#include <vector>
#include <stdlib.h>
#include "types.hpp"
#include <atomic>
#include <thread>
#include "workers.hpp"


DetectionWorker::DetectionWorker(SharedLatest &shared,
                std::atomic<bool> &stop_flag)
    : shared_(shared), stop_(stop_flag)
{
    start_time_        = Clock::now();
    selected_robot_id_ = -1;
    ttl_               = 0.0f;
    initial_yaw_       = 0.0f;
    last_cam_ver_      = 0;
}

void DetectionWorker::operator()() {
    while (!stop_.load(std::memory_order_relaxed)) {

        uint64_t cur_ver = shared_.camera_ver.load(std::memory_order_relaxed);
        if (cur_ver == last_cam_ver_) {
            sleep_small();
            continue; // no new camera frame
        }
        last_cam_ver_ = cur_ver;

        auto cam = std::atomic_load(&shared_.camera);
        if (!cam) {
            sleep_small();
            continue;
        }

        // snapshot IMU (ref only)
        auto imu = std::atomic_load(&shared_.imu);

        // 1) YOLO
        std::vector<DetectionResult> dets =
            yolo_predict(cam->raw_data, cam->width, cam->height);

        // 2) keypoint refine + filtering by confidence
        refine_keypoints(dets, cam->width, cam->height);

        // 3) solvePnP + yaw in cam frame
        solvepnp_and_yaw(dets);

        // 4) transform to world using latest IMU
        if (imu) {
            transform_to_world(dets, *imu);
        }

        // 5) group + select
        auto grouped = group_armors(dets);
        auto selected = select_armor(grouped,
                                        ttl_,
                                        selected_robot_id_,
                                        initial_yaw_);

        // 6) form RobotState
        auto robot = form_robot(selected);
        if (robot) {
            robot->timestamp = cam->timestamp;
            auto ptr = std::make_shared<RobotState>(*robot);
            std::atomic_store(&shared_.detection_out, ptr);
            shared_.detection_ver.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

void DetectionWorker::sleep_small() {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

std::vector<DetectionResult>
DetectionWorker::yolo_predict(const std::vector<uint8_t> &raw, int w, int h) {
    return {};
}

void DetectionWorker::refine_keypoints(std::vector<DetectionResult> &dets, int w, int h) {
    // trad CV refine
}

void DetectionWorker::solvepnp_and_yaw(std::vector<DetectionResult> &dets) {
    // solvePnP, Rodrigues, decompose, yaw_rad
}

void DetectionWorker::transform_to_world(std::vector<DetectionResult> &dets,
                        const IMUState &imu) {
    // R_cam2world using imu yaw/pitch, apply to tvec
}

std::vector<std::vector<DetectionResult>>
DetectionWorker::group_armors(const std::vector<DetectionResult> &dets) {
    return {};
}

std::vector<DetectionResult>
DetectionWorker::select_armor(const std::vector<std::vector<DetectionResult>> &grouped,
                float &ttl, int &selected_robot_id, float &initial_yaw) {
    return {};
}

std::unique_ptr<RobotState>
DetectionWorker::form_robot(const std::vector<DetectionResult> &armors) {
    return nullptr;
}
