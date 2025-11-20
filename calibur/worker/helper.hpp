#include <vector>
#include <Eigen/Dense>
#include "types.hpp"

using namespace std;

constexpr float PI     = static_cast<float>(M_PI);
constexpr float HALF_PI = 0.5f * PI;
constexpr float QUARTER_PI = 0.25f * PI;
constexpr float TWO_PI  = 2.0f * PI;

inline float wrap_pi(const float angle) {
    return std::fmod(angle + M_PI, 2.0f * M_PI) - M_PI;
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

inline Eigen::Matrix3f make_R_cam2world_from_yaw_pitch(float yaw_cam_world,
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

    Eigen::Matrix3f R_cam2world = Ryaw * Rpitch;
    return R_cam2world;
}