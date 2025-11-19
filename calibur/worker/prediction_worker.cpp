#include <atomic>
#include <vector>
#include <cmath>
#include <thread>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "types.hpp"
#include "workers.hpp"
#include "helper.hpp"


PredictionWorker::PredictionWorker(SharedLatest &shared,
                    SharedScalars &scalars,
                    std::atomic<bool> &stop_flag)
    : shared_(shared), scalars_(scalars), stop_(stop_flag), last_pf_ver_(0) {}

void PredictionWorker::operator()() {
    while (!stop_.load(std::memory_order_relaxed)) {
        uint64_t cur_ver = shared_.pf_ver.load(std::memory_order_relaxed);
        if (cur_ver == last_pf_ver_) {
            sleep_small();
            continue; // no new PF state
        }
        last_pf_ver_ = cur_ver;

        auto pf = std::atomic_load(&shared_.pf_out);
        if (!pf) {
            continue;
        }

        auto imu = std::atomic_load(&shared_.imu);
        float measured_speed = scalars_.bullet_speed.load(std::memory_order_relaxed);

        PredictionOut out{};
        compute_prediction(*pf, imu.get(), measured_speed, out);

        auto ptr = std::make_shared<PredictionOut>(out);
        std::atomic_store(&shared_.prediction_out, ptr);
        shared_.prediction_ver.fetch_add(1, std::memory_order_relaxed);
    }
}

void PredictionWorker::sleep_small() {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void PredictionWorker::compute_prediction(const RobotState &rs,
                        const IMUState *imu,
                        float measured_speed,
                        PredictionOut &out)
{
    // Reuse small buffers instead of reallocating every call
    static thread_local std::vector<float> tvec;
    static thread_local std::vector<float> pos_lead;
    static thread_local std::vector<float> correction;

    // --- bullet_speed filtering ---
    float bullet_speed = this->bullet_speed; // keep in a register
    filtering(bullet_speed, measured_speed, ALPHA_BULLET_SPEED);
    this->bullet_speed = bullet_speed;

    // --- t_processing_time filtering ---
    const auto now = Clock::now();
    const float proc_time =
        std::chrono::duration_cast<std::chrono::duration<float>>(now - rs.timestamp).count();

    float processing_time = this->processing_time;
    filtering(processing_time, proc_time, ALPHA_PROCESSING_TIME);
    this->processing_time = processing_time;

    // --- convergence method for motion model (WORLD frame) ---
    tvec.resize(3);
    tvec[0] = rs.state[0];
    tvec[1] = rs.state[1];
    tvec[2] = rs.state[2];

    float t_lead = t_lead_calculation(tvec, bullet_speed)
                 + processing_time
                 + this->t_gimbal_actuation;

    int iter = 0;
    constexpr int MAX_ITERS = PRED_CONV_MAX_ITERS;

    do {
        pos_lead = motion_model_robot(rs.state, t_lead);
        t_lead = t_lead_calculation(pos_lead, bullet_speed)
               + processing_time
               + this->t_gimbal_actuation;
        ++iter;
    } while (!is_converged(t_lead, PREDICTION_CONVERGENCE_THRESHOLD) &&
             iter < MAX_ITERS);

    // --- world2cam using imu yaw/pitch ---
    Eigen::Matrix3f R_world2cam = world2cam(this->shared_);
    pos_lead = pos_world2cam(pos_lead, R_world2cam);
    //TODO: convert yaw from world to cam for better yaw accuracy

    // --- bullet drop correction ---
    {
        const float x = pos_lead[0];
        const float y = pos_lead[1];
        const float z = pos_lead[2];
        const float distance = std::sqrt(x * x + y * y + z * z);

        const float drop_correction = bullet_drop_correction(distance, bullet_speed);
        pos_lead[1] += drop_correction;
    }

    correction = calculate_gimbal_correction(pos_lead);
    const bool fire_state  = should_fire(pos_lead);
    
    const bool chase_state = (pos_lead[2] > CHASE_THREASHOLD);
    const bool aim_state   = true;  // TODO: hook to PF state machine

    this->fire_state  = fire_state;
    this->chase_state = chase_state;
    this->aim_state   = aim_state;
}

inline void filtering(float &value, float measurement, float alpha) {
    const float one_minus_alpha = 1.0f - alpha;
    value = alpha * measurement + one_minus_alpha * value;
}

inline bool is_converged(float v, float threshold) {
    return std::fabs(v) < threshold;
}

inline float t_lead_calculation(const std::vector<float> &tvec, const float &bullet_speed) {
    const float dx = tvec[0];
    const float dy = tvec[1];
    const float dz = tvec[2];

    const float distance2 = dx * dx + dy * dy + dz * dz;
    const float distance  = std::sqrt(distance2);

    // Caller should ensure bullet_speed > 0
    return distance / bullet_speed;
}

inline int sector_from_yaw(const float yaw) {
    // Precompute constants once per TU, not per call
    constexpr float HALF_PI    = static_cast<float>(M_PI_2);
    constexpr float QUARTER_PI = static_cast<float>(M_PI_4);
    const float theta = wrap_pi(yaw);
    const float sector_f = (theta + QUARTER_PI) / HALF_PI;
    return static_cast<int>(std::floor(sector_f)) & 3;
}

inline std::vector<float> motion_model_robot(const std::vector<float> &state, const float &t) {
    // Assume state.size() >= 15
    std::vector<float> final_pos(3);
    const float tt   = t * t;
    const float half = 0.5f;
    final_pos[0] = state[0] + state[3] * t + half * state[6] * tt; // x
    final_pos[1] = state[1] + state[4] * t + half * state[7] * tt; // y
    final_pos[2] = state[2] + state[5] * t + half * state[8] * tt; // z

    // yaw with yaw_rate and yaw_acc
    const float yaw = state[9] + state[10] * t + half * state[11] * tt;
    const int armor_plate_idx = sector_from_yaw(yaw);
    constexpr float TWO = 2.0f;
    constexpr float QUARTER_PI = static_cast<float>(M_PI_4); // pi/4
    constexpr float HALF_PI    = static_cast<float>(M_PI_2); // pi/2
    const float yaw_shifted  = yaw + QUARTER_PI;
    const float yaw_restrict = std::fmod(yaw_shifted, TWO * HALF_PI) - QUARTER_PI;
    const float radius = armor_plate_idx ? state[13] : state[12];
    const float s = std::sin(yaw_restrict);
    const float c = std::cos(yaw_restrict);
    final_pos[0] += radius * s;
    final_pos[2] -= radius * c;
    // height offset
    final_pos[1] += state[14];
    return final_pos;
}

inline std::vector<float> calculate_gimbal_correction(const std::vector<float> &tvec) {
    std::vector<float> correction(2);

    const float x = tvec[0];
    const float y = tvec[1];
    const float z = tvec[2];
    correction[0] = std::atan2(x, z);
    correction[1] = std::atan2(y, z);

    return correction;
}

inline float bullet_drop_correction(const float &distance, float bullet_speed) {
    constexpr float G = 9.81f;
    // Use inverse speed to turn division into multiply (often cheaper)
    const float inv_v  = 1.0f / bullet_speed;
    const float inv_v2 = inv_v * inv_v;

    return 0.5f * G * distance * distance * inv_v2;
}

inline int should_fire(const std::vector<float> &tvec) {
    // Assume tvec.size() >= 2 (tvec[0]=x, tvec[1]=y in angle or pixel units)
    constexpr float HALF = 0.5f;
    const float x_tolerance = WIDTH_TOLERANCE  * TOLERANCE_COEFF * HALF;
    const float y_tolerance = HEIGHT_TOLERANCE * TOLERANCE_COEFF * HALF;
    const float ax = std::fabs(tvec[0]);
    const float ay = std::fabs(tvec[1]);

    return (ax < x_tolerance) && (ay < y_tolerance);
}






