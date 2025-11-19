#pragma once

#include <atomic>
#include <memory>
#include <vector>

#include "types.hpp"


// ------------------------------------------- Constants -------------------------------------------
#define ALPHA_BULLET_SPEED                      0.1f
#define ALPHA_PROCESSING_TIME                   0.1f
#define PREDICTION_CONVERGENCE_THRESHOLD        0.01f
#define CHASE_THREASHOLD                        6.0f
#define PRED_CONV_MAX_ITERS                     10
#define WIDTH_TOLERANCE                         0.13f  // meters
#define HEIGHT_TOLERANCE                        0.13f  // meters
#define TOLERANCE_COEFF                         1.0f   // empirical scaling

//--------------------------------------------Camera Worker--------------------------------------------

class CameraWorker {
public:
    CameraWorker(void* cam_handle,
                 SharedLatest &shared,
                 std::atomic<bool> &stop_flag);

    // Long-running loop for thread pool
    void operator()();

private:
    void*           cam_;
    SharedLatest   &shared_;
    std::atomic<bool> &stop_;

    // You can replace this with a real SDK call in the .cpp
    void grab_frame_stub(CameraFrame &frame);
};

//--------------------------------------------IMU Worker--------------------------------------------

class IMUWorker {
public:
    IMUWorker(SharedLatest &shared,
              std::atomic<bool> &stop_flag);

    void operator()();

private:
    SharedLatest    &shared_;
    std::atomic<bool> &stop_;

    // Replace with real IMU driver in .cpp
    bool read_imu_stub(IMUState &imu);
};


//--------------------------------------------Detection Worker--------------------------------------------

class DetectionWorker {
public:
    DetectionWorker(SharedLatest &shared,
                    std::atomic<bool> &stop_flag);

    void operator()();

private:
    SharedLatest    &shared_;
    std::atomic<bool> &stop_;

    // "class variables"
    TimePoint   start_time_;
    int         selected_robot_id_;
    float       ttl_;
    float       initial_yaw_;
    uint64_t    last_cam_ver_;

    void sleep_small();

    // ---- Interfaces for you to implement in .cpp ----

    std::vector<DetectionResult>
    yolo_predict(const std::vector<uint8_t> &raw, int width, int height);

    void refine_keypoints(std::vector<DetectionResult> &dets,
                          int width, int height);

    void solvepnp_and_yaw(std::vector<DetectionResult> &dets);

    void transform_to_world(std::vector<DetectionResult> &dets,
                            const IMUState &imu);

    std::vector<std::vector<DetectionResult>>
    group_armors(const std::vector<DetectionResult> &dets);

    std::vector<DetectionResult>
    select_armor(const std::vector<std::vector<DetectionResult>> &grouped,
                 float &ttl,
                 int &selected_robot_id,
                 float &initial_yaw);

    std::unique_ptr<RobotState>
    form_robot(const std::vector<DetectionResult> &armors);
};


//--------------------------------------------PF Worker--------------------------------------------

class PFWorker {
public:
    PFWorker(SharedLatest &shared,
             std::atomic<bool> &stop_flag);

    // Runs as dedicated thread (not via pool)
    void operator()();

private:
    SharedLatest    &shared_;
    std::atomic<bool> &stop_;
    uint64_t        last_det_ver_;

    // PF / CUDA interfaces to implement in .cpp
    void gpu_pf_init();
    void gpu_pf_reset(const RobotState &meas);
    RobotState gpu_pf_predict_only();
    RobotState gpu_pf_step(const RobotState &meas);
};

//--------------------------------------------Prediction Worker--------------------------------------------

class PredictionWorker {
public:
    PredictionWorker(SharedLatest &shared,
                     SharedScalars &scalars,
                     std::atomic<bool> &stop_flag);

    void operator()();

private:
    SharedLatest    &shared_;
    SharedScalars   &scalars_;
    std::atomic<bool> &stop_;
    uint64_t        last_pf_ver_;
    float           bullet_speed = 20.0f; 
    float           processing_time = 0.05f;
    float           t_gimbal_actuation = 0.1f;
    int            fire_state = false;
    int            chase_state = false;
    int            aim_state = false;

    void sleep_small();

    void compute_prediction(const RobotState &rs,
                            const IMUState *imu,
                            float bullet_speed,
                            PredictionOut &out);

    double time_to_double(const TimePoint &tp);
};

//--------------------------------------------Prediction Worker--------------------------------------------

class USBWorker {
public:
    USBWorker(SharedLatest &shared,
              SharedScalars &scalars,
              std::atomic<bool> &stop_flag);

    void operator()();

private:
    SharedLatest    &shared_;
    SharedScalars   &scalars_;
    std::atomic<bool> &stop_;
    uint64_t        last_pred_ver_;

    void process_usb_rx();
    void usb_send_tx(const PredictionOut &out);
};


