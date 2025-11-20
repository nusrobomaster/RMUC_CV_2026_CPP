#include <cmath>
#include <vector>
#include <stdlib.h>
#include <atomic>
#include <thread>

#include "workers.hpp"
#include "types.hpp"
#include "helper.hpp"

using RobotStatePtr = std::shared_ptr<const RobotState>;

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

    static thread_local float imu_yaw, imu_pitch;
    static thread_local RobotState local_robot_state;   //keep track of what robot is being tracked
    static thread_local std::shared_ptr<IMUState> imu;
    static thread_local std::vector<DetectionResult> selected_armors;
    static thread_local std::vector<DetectionResult> dets;
    static thread_local std::vector<std::vector<DetectionResult>> grouped_armors;
    //static thread_local RobotState robot;


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
        imu = std::atomic_load(&shared_.imu);

        // 1) YOLO
        yolo_predict(cam->raw_data, cam->width, cam->height, dets);

        // 2) keypoint refine + filtering by confidence
        refine_keypoints(dets, cam->width, cam->height);

        // 3) solvePnP + yaw in cam frame
        solvepnp_and_yaw(dets);

        // 4) group + select
        group_armors(dets, grouped_armors);
        select_armor(grouped_armors, ttl_, selected_robot_id_, initial_yaw_, selected_armors);

        // 5) transform to world using latest IMU
        //TODO: considering whether to do pnp first or select robot first
        bool success = get_imu_yaw_pitch(this->shared_, imu_yaw, imu_pitch);
        //if success: ...
        const Eigen::Matrix3f R_world2cam = make_R_cam2world_from_yaw_pitch(imu_yaw, imu_pitch);
        for (auto &det : selected_armors) {
            cam2world(det, R_world2cam, imu_yaw, imu_pitch);
        }

        // 6) form RobotState
        auto robot = form_robot(selected_armors);
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
DetectionWorker::yolo_predict(const std::vector<uint8_t> &raw, int w, int h, std::vector<DetectionResult> &dets) {
    return {};
}

void DetectionWorker::refine_keypoints(std::vector<DetectionResult> &dets, int w, int h) {
    // trad CV refine
}

void DetectionWorker::solvepnp_and_yaw(std::vector<DetectionResult> &dets) {
    // solvePnP, Rodrigues, decompose, yaw_rad

    //need to convert cv::Vec3f to Eigen::Vector3f here

}


void DetectionWorker::group_armors(const std::vector<DetectionResult> &dets,
                                    std::vector<std::vector<DetectionResult>> &grouped_armors) {

}

void DetectionWorker::select_armor(const std::vector<std::vector<DetectionResult>> &grouped_armors,
                 float &ttl,
                 int &selected_robot_id,
                 float &initial_yaw,
                 std::vector<DetectionResult> &selected_armors) {

    constexpr float MAX_TTL = SELECTOR_TTL;
    float dt = 0.02f;                  // or compute real dt

    // NO DETECTIONS
    if (grouped_armors.empty()) {
        ttl -= dt;
        if (ttl <= 0) {
            selected_robot_id = -1;
            selected_armors.clear();
        }
        return;
    }

    // NO TARGET CURRENTLY SELECTED
    if (selected_robot_id & 0x80000000) {   // id < 0
        selected_robot_id = choose_best_robot(grouped_armors);
        selected_armors = grouped_armors[selected_robot_id];
        ttl = MAX_TTL;
        return;
    }

    // CHECK IF CURRENTLY TRACKED ROBOT IS SEEN
    int tracked_idx = -1;
    for (int i = 0; i < grouped_armors.size(); i++) {
        if (grouped_armors[i][0].class_id == selected_robot_id) {
            tracked_idx = i;
            break;
        }
    }

    if (tracked_idx != -1) {
        selected_armors = grouped_armors[tracked_idx];
        ttl = MAX_TTL;
        return;            // continue tracking same robot
    }

    // TRACKED ROBOT MISSING → GRACE PERIOD
    ttl -= dt;
    if (ttl > 0) {
        selected_armors.clear();   // missing 1–N frames, but tracking continues
        return;
    }

    // FULL LOST → SWITCH TARGET
    selected_robot_id = choose_best_robot(grouped_armors);
    selected_armors = grouped_armors[selected_robot_id];
    ttl = MAX_TTL;

}

std::unique_ptr<RobotState>
DetectionWorker::form_robot(const std::vector<DetectionResult> &armors) {
    if (armors.empty() || armors.size() > 2) {
        if (!this->has_prev_robot_) {
            // No previous state yet → nothing meaningful to return
            return nullptr;
        }
        // Return a copy of the previous robot state
        return std::make_unique<RobotState>(prev_robot_);
    }
    auto rs = std::make_unique<RobotState>();
    if (armors.size() == 1) {
        this->has_prev_robot_ = from_one_armor(armors[0], prev_robot_, this->has_prev_robot_);
    } else {
        this->has_prev_robot_ = from_two_armors(armors[0], armors[1], prev_robot_, this->has_prev_robot_);
    }

    return rs;
}

inline void cam2world(DetectionResult &det, const Eigen::Matrix3f &R_world2cam,
                     const float imu_yaw, const float imu_pitch) {
    det.tvec = R_world2cam * det.tvec;
    det.yaw_rad += imu_yaw;
}

// Compute Euclidean distance quickly
inline float tvec_distance(const Eigen::Vector3f& t) {
    return std::sqrt(t[0]*t[0] + t[1]*t[1] + t[2]*t[2]);
}

// Choose robot with minimum average distance of its armors
int choose_best_robot(const std::vector<std::vector<DetectionResult>>& grouped_armors)
{
    int best_idx = 0;
    float best_dist = std::numeric_limits<float>::max();

    for (int i = 0; i < grouped_armors.size(); i++) {
        const auto& g = grouped_armors[i];

        float avg_dist;
        if (g.size() == 1) {
            avg_dist = tvec_distance(g[0].tvec);
        } else {
            // two armors → average the distance
            float d1 = tvec_distance(g[0].tvec);
            float d2 = tvec_distance(g[1].tvec);
            avg_dist = 0.5f * (d1 + d2);
        }

        if (avg_dist < best_dist) {
            best_dist = avg_dist;
            best_idx = i;
        }
    }

    return best_idx;
}

// based on the difference between armor yaw and previous yaw, decide the sector the armor yaw is in
// therefore decide what is the actual robot yaw
// TODO: check correctness
inline int choose_yaw_sector(float yaw){
    float angle = std::fmod(yaw + PI, TWO_PI);
    if (angle < 0.0f) angle += TWO_PI;

    angle += QUARTER_PI;
    if (angle >= TWO_PI) angle -= TWO_PI;
    int sector = static_cast<int>(angle / HALF_PI);
    return sector;
}

inline bool from_one_armor(const DetectionResult &det, RobotState &robot, bool valid) {
    if (!valid) {
        robot.state[IDX_R1]   = DEFAULT_ROBOT_RADIUS;
        robot.state[IDX_R2]   = DEFAULT_ROBOT_RADIUS;
        robot.state[IDX_YAW]  = det.yaw_rad;
    } 

    const float r1 = robot.state[IDX_R1];
    const float r2 = robot.state[IDX_R2];
    const float armor_yaw = det.yaw_rad;

    float chosen_yaw = robot.state[IDX_YAW];
    float r          = r1;  // default to r1

    // If we already had a yaw, refine it like the Python version
    if (valid) {
        const float prev_yaw = robot.state[IDX_YAW];

        // Candidate yaws: prev, prev±π/2, prev+π
        float best_yaw = prev_yaw;
        float best_err = std::fabs(wrap_pi(prev_yaw - armor_yaw));

        const float candidates[3] = {
            prev_yaw + HALF_PI,
            prev_yaw - HALF_PI,
            prev_yaw + PI
        };

        for (float cand : candidates) {
            const float err = std::fabs(wrap_pi(cand - armor_yaw));
            if (err < best_err) {
                best_err = err;
                best_yaw = wrap_pi(cand);
            }
        }

        chosen_yaw = best_yaw;

        // Sector selection (same logic as Python)
        float angle = chosen_yaw + PI;           // [0, 2π) via mod
        angle = std::fmod(angle, TWO_PI);
        if (angle < 0.0f) angle += TWO_PI;
        float angle_shifted = angle + QUARTER_PI;
        angle_shifted = std::fmod(angle_shifted, TWO_PI);
        if (angle_shifted < 0.0f) angle_shifted += TWO_PI;
        const int sector = static_cast<int>(std::floor(angle_shifted / HALF_PI));
        r = (sector & 1) ? r2 : r1;  // odd sector → r2, even → r1
    }

    // Update yaw in the state
    robot.state[IDX_YAW] = chosen_yaw;

    // Position: use armor yaw for sin/cos (same as original Python)
    const float s = std::sin(armor_yaw);
    const float c = std::cos(armor_yaw);

    robot.state[IDX_TX] = det.tvec[0] - r * s;
    robot.state[IDX_TY] = det.tvec[1];
    robot.state[IDX_TZ] = det.tvec[2] + r * c;
    robot.class_id = det.class_id;

    return true;

}

inline bool from_two_armors(const DetectionResult &det1, const DetectionResult &det2,
                           RobotState &robot, bool valid) {

}
