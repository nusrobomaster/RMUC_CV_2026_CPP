#ifndef HELPER_HPP
#define HELPER_HPP

#include <vector>
#include <Eigen/Dense>
#include "types.hpp"

using namespace std;

constexpr float PI     = static_cast<float>(M_PI);
constexpr float HALF_PI = 0.5f * PI;
constexpr float QUARTER_PI = 0.25f * PI;
constexpr float TWO_PI  = 2.0f * PI;

constexpr float GIMBAL_PITCH_MIN = -0.17f;  // ~-10° (looking down) - ADJUST THIS
constexpr float GIMBAL_PITCH_MAX =  0.87f;  // ~+50° (looking up)   

// Yaw limits - Full 360° rotation (no limits)
constexpr float GIMBAL_YAW_MIN = -3.14f;    // -180°
constexpr float GIMBAL_YAW_MAX =  3.14f;    // +180°

// Software safety margin (stay away from hard limits)
constexpr float GIMBAL_SAFETY_MARGIN = 0.05f;  // ~3 degrees

// For 360° gimbal, yaw wrapping is more important than limits
constexpr bool GIMBAL_HAS_YAW_LIMITS = false;  // Set to true if yaw is limited

inline float wrap_pi(const float angle) {
    return std::fmod(angle + M_PI, 2.0f * M_PI) - M_PI;
}

inline float deg2rad(float deg) {
    return deg * (PI / 180.0f);
}

inline bool get_imu_yaw_pitch(const SharedLatest &shared,
                              float &yaw_cam_world,
                              float &pitch_cam_world) {
    std::shared_ptr<IMUState> imu_ptr = std::atomic_load(&shared.imu);
    if (!imu_ptr) {
        return false;
    }

    const IMUState &imu = *imu_ptr;

    if (imu.euler_angle.size() < 3) {
        return false;
    }
    // euler_angle = {roll, pitch, yaw} in *world frame*
    // float roll_deg  = imu.euler_angle[0];
    float pitch_deg = imu.euler_angle[1];
    float yaw_deg   = imu.euler_angle[2];


    pitch_cam_world = deg2rad(pitch_deg);
    yaw_cam_world   = deg2rad(yaw_deg);

    return true;
}

inline Eigen::Matrix3f make_R_cam2world_from_yaw_pitch(float yaw_cam_world,
                                                        float pitch_cam_world)
{
    // - Positive pitch = camera tilts DOWN
    // - Positive yaw = camera turns LEFT
    // - Negative yaw = camera turns RIGHT
    
    // Camera frame: X=right, Y=down, Z=forward (standard OpenCV/computer vision)
    // World frame: Need to determine, but rotations are relative to camera orientation
    
    // Rotation around X-axis (pitch)
    // Positive pitch = tilting down
    Eigen::Matrix3f Rpitch;
    Rpitch << 1.0f,                  0.0f,                   0.0f,
              0.0f,  std::cos(pitch_cam_world), -std::sin(pitch_cam_world),
              0.0f,  std::sin(pitch_cam_world),  std::cos(pitch_cam_world);
    
    // Rotation around Y-axis (yaw)
    // Based on your IMU data, the standard Y-axis rotation matrix should work
    // Standard form for Y-axis rotation (right-hand rule):
    Eigen::Matrix3f Ryaw;
    // Ryaw <<  std::cos(yaw_cam_world), 0.0f, std::sin(yaw_cam_world),
    //          0.0f,                    1.0f, 0.0f,
    //         -std::sin(yaw_cam_world), 0.0f, std::cos(yaw_cam_world);

    Ryaw <<  std::cos(yaw_cam_world), -std::sin(yaw_cam_world),     0.0f,
             std::sin(yaw_cam_world), std::cos(yaw_cam_world),      0.0f,
             0.0f                   , 0.0f                      ,   1.0f;
    
    // Build cam2world: apply pitch first (camera tilt), then yaw (camera pan)
    Eigen::Matrix3f R_cam2world = Ryaw * Rpitch;
    
    // Return world2cam (inverse = transpose)
    return R_cam2world.transpose();
}


inline Eigen::Matrix3f make_R_world2cam_from_yaw_pitch(float yaw_cam_world,
                                                        float pitch_cam_world)
{
    Eigen::Matrix3f R_cam2world = make_R_cam2world_from_yaw_pitch(yaw_cam_world, pitch_cam_world);
    return R_cam2world.transpose();
}

inline void clamp_to_gimbal_limits(float &yaw, float &pitch) {
    // Clamp pitch with safety margin
    const float pitch_min_safe = GIMBAL_PITCH_MIN + GIMBAL_SAFETY_MARGIN;
    const float pitch_max_safe = GIMBAL_PITCH_MAX - GIMBAL_SAFETY_MARGIN;
    pitch = std::clamp(pitch, pitch_min_safe, pitch_max_safe);
    
    // For 360° gimbal, wrap yaw instead of clamping
    if (!GIMBAL_HAS_YAW_LIMITS) {
        // Wrap yaw to [-π, π]
        yaw = wrap_pi(yaw);
    } else {
        // Clamp yaw if there are physical limits
        const float yaw_min_safe = GIMBAL_YAW_MIN + GIMBAL_SAFETY_MARGIN;
        const float yaw_max_safe = GIMBAL_YAW_MAX - GIMBAL_SAFETY_MARGIN;
        yaw = std::clamp(yaw, yaw_min_safe, yaw_max_safe);
    }
}

inline bool is_at_pitch_limit(float pitch) {
    const float tolerance = 0.08f;  // ~4.5 degrees from limit
    
    return (pitch < GIMBAL_PITCH_MIN + tolerance) || 
           (pitch > GIMBAL_PITCH_MAX - tolerance);
}

inline bool is_target_reachable(float yaw, float pitch) {
    // Yaw is always reachable for 360° gimbal
    // Only check pitch
    return (pitch >= GIMBAL_PITCH_MIN && pitch <= GIMBAL_PITCH_MAX);
}

inline void get_gimbal_status_string(float yaw, float pitch, char* buffer, size_t bufsize) {
    const char* pitch_status = "";
    if (pitch < GIMBAL_PITCH_MIN + 0.08f) {
        pitch_status = " [AT MIN]";
    } else if (pitch > GIMBAL_PITCH_MAX - 0.08f) {
        pitch_status = " [AT MAX]";
    }
    
    snprintf(buffer, bufsize, "yaw=%.2f° pitch=%.2f°%s", 
             yaw * 180.0f / M_PI, 
             pitch * 180.0f / M_PI,
             pitch_status);
}


// inline Eigen::Matrix3f make_R_cam2world_from_yaw_pitch(float yaw_cam_world,
//                                                         float pitch_cam_world)
// {
//     Eigen::Matrix3f Rpitch;
//     Rpitch << 1.0f,                  0.0f,                   0.0f,
//               0.0f,  std::cos(pitch_cam_world), -std::sin(pitch_cam_world),
//               0.0f,  std::sin(pitch_cam_world),  std::cos(pitch_cam_world);
    
//     Eigen::Matrix3f Ryaw;
//     Ryaw <<  std::cos(yaw_cam_world), 0.0f, std::sin(yaw_cam_world),
//              0.0f,                    1.0f, 0.0f,
//             -std::sin(yaw_cam_world), 0.0f, std::cos(yaw_cam_world);
    
//     Eigen::Matrix3f R_cam2world = Ryaw * Rpitch;
    
//     return R_cam2world;
// }

#endif