#include <vector>
#include <Eigen/Dense>
#include "types.hpp"

using namespace std;

inline float wrap_pi(const float angle) {
    return std::fmod(angle + M_PI, 2.0f * M_PI) - M_PI;
}

inline Eigen::Matrix3f cam2world(const SharedLatest &shared) {
    // Read latest IMU safely (lock-free)
    std::shared_ptr<IMUState> imu_ptr = std::atomic_load(&shared.imu);
    if (!imu_ptr) {
        return Eigen::Matrix3f::Identity();
    }

    const IMUState &imu = *imu_ptr;

    // Assuming IMUState::euler_angle = {roll, pitch, yaw} (radians)
    float pitch = imu.euler_angle[1];
    float yaw   = imu.euler_angle[2];

    // ---- cam → world (same as your Python) ----
    Eigen::Matrix3f Rpitch;
    Rpitch << 1.0f,            0.0f,             0.0f,
              0.0f,  std::cos(pitch), -std::sin(pitch),
              0.0f,  std::sin(pitch),  std::cos(pitch);

    Eigen::Matrix3f Ryaw;
    Ryaw <<  std::cos(yaw), 0.0f, std::sin(yaw),
             0.0f,          1.0f, 0.0f,
            -std::sin(yaw), 0.0f, std::cos(yaw);

    Eigen::Matrix3f R_cam2world = Ryaw * Rpitch;

    return R_cam2world;
}

inline Eigen::Matrix3f world2cam(const SharedLatest &shared) {
    Eigen::Matrix3f R_cam2world = cam2world(shared);
    return R_cam2world.transpose();
}

inline bool get_imu_yaw_pitch(const SharedLatest &shared,
                              float &yaw_cam_world,
                              float &pitch_cam_world) {
    std::shared_ptr<IMUState> imu_ptr = std::atomic_load(&shared.imu);
    if (!imu_ptr) {
        return false;
    }

    const IMUState &imu = *imu_ptr;
    // euler_angle = {roll, pitch, yaw} in *world frame*
    pitch_cam_world = imu.euler_angle[1];
    yaw_cam_world   = imu.euler_angle[2];
    return true;
}

inline Eigen::Matrix3f make_R_world2cam_from_yaw_pitch(float yaw_cam_world,
                                                       float pitch_cam_world)
{
    Eigen::Matrix3f Rpitch;
    Rpitch << 1.0f,                  0.0f,                   0.0f,
              0.0f,  std::cos(pitch_cam_world), -std::sin(pitch_cam_world),
              0.0f,  std::sin(pitch_cam_world),  std::cos(pitch_cam_world);

    Eigen::Matrix3f Ryaw;
    Ryaw <<  std::cos(yaw_cam_world), 0.0f, std::sin(yaw_cam_world),
             0.0f,                    1.0f, 0.0f,
            -std::sin(yaw_cam_world), 0.0f, std::cos(yaw_cam_world);

    Eigen::Matrix3f R_cam2world = Ryaw * Rpitch;   // cam → world
    return R_cam2world.transpose();                // world → cam
}


inline std::vector<float> pos_world2cam(const std::vector<float> &pos_world,
                                    const Eigen::Matrix3f &R_world2cam)
{
    Eigen::Map<const Eigen::Vector3f> pW(pos_world.data());
    Eigen::Vector3f pC = R_world2cam * pW;

    return {pC.x(), pC.y(), pC.z()};
}
