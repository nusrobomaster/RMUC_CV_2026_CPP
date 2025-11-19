#include "types.hpp"
#include <thread>
#include "workers.hpp"
#include "rbpf.cuh"

static constexpr int NUM_PARTICLES = 10000;

using clock_t = std::chrono::steady_clock;

PFWorker::PFWorker(SharedLatest &shared,
                   std::atomic<bool> &stop_flag)
    : shared_(shared),
      stop_(stop_flag),
      last_det_ver_(0) {}

void PFWorker::gpu_pf_init() {
    if (!g_pf) {
        constexpr int NUM_PARTICLES = 10000;
        g_pf = rbpf_create(NUM_PARTICLES);
        // Optionally reset from some initial guess
        RobotState init{};
        rbpf_reset_from_meas(g_pf, init);
    }
}

void PFWorker::gpu_pf_reset(const RobotState &meas) {
    rbpf_reset_from_meas(g_pf, meas);
}

RobotState PFWorker::gpu_pf_predict_only() {
    rbpf_predict(g_pf, kDt);
    return rbpf_get_mean(g_pf);
}

RobotState PFWorker::gpu_pf_step(const RobotState &meas) {
    rbpf_step(g_pf, meas, kDt);
    return rbpf_get_mean(g_pf);
}

void PFWorker::operator()() {
    gpu_pf_init();

    auto next_tick = clock_t::now();

    while (!stop_.load(std::memory_order_acquire)) {
        // ---- wait until next 10 ms slot ----
        next_tick += std::chrono::milliseconds(10);
        std::this_thread::sleep_until(next_tick);

        // ---- check if there is a new detection ----
        bool            has_meas = false;
        RobotState      meas;
        uint64_t        det_ver  = shared_.detection_ver.load(std::memory_order_acquire);

        if (det_ver != last_det_ver_) {
            auto det_ptr = shared_.detection_out;   // shared_ptr copy is thread–safe
            if (det_ptr) {
                meas           = *det_ptr;
                last_det_ver_  = det_ver;
                has_meas       = true;
            }
        }

        // ---- run PF step ----
        RobotState pf_state;
        if (has_meas) {
            pf_state = gpu_pf_step(meas);      // predict + update for this 0.01 s frame
        } else {
            pf_state = gpu_pf_predict_only();  // predict only for this 0.01 s frame
        }

        // ---- publish PF result ----
        shared_.pf_out = std::make_shared<RobotState>(pf_state);
        shared_.pf_ver.fetch_add(1, std::memory_order_acq_rel);
    }
}


