#include "types.hpp"
#include <thread>
#include "workers.hpp"

PFWorker::PFWorker(SharedLatest &shared,
            std::atomic<bool> &stop_flag)
    : shared_(shared), stop_(stop_flag), last_det_ver_(0)
{
    gpu_pf_init();
}

void PFWorker::operator()() {
    using namespace std::chrono;
    const auto period = milliseconds(10);
    auto next = Clock::now();

    while (!stop_.load(std::memory_order_relaxed)) {
        next += period;

        uint64_t cur_ver = shared_.detection_ver.load(std::memory_order_relaxed);
        bool has_new_det = (cur_ver != last_det_ver_);
        std::shared_ptr<RobotState> det_ptr;

        if (has_new_det) {
            last_det_ver_ = cur_ver;
            det_ptr = std::atomic_load(&shared_.detection_out);
        }

        RobotState out;
        if (!det_ptr) {
            out = gpu_pf_predict_only();
        } else {
            if (det_ptr->pf_state == PF_STATE_RESET) {
                gpu_pf_reset(*det_ptr);
            }
            out = gpu_pf_step(*det_ptr);
        }

        out.timestamp = Clock::now();

        auto ptr = std::make_shared<RobotState>(std::move(out));
        std::atomic_store(&shared_.pf_out, ptr);
        shared_.pf_ver.fetch_add(1, std::memory_order_relaxed);

        std::this_thread::sleep_until(next);
    }
}

// --- You implement these with CUDA/CPU ---
void PFWorker::gpu_pf_init() {
    // cudaFree(0), cudaMalloc, etc.
}

void PFWorker::gpu_pf_reset(const RobotState &meas) {
    // reinit particle set
}

RobotState PFWorker::gpu_pf_predict_only() {
    RobotState rs;
    return rs;
}

RobotState PFWorker::gpu_pf_step(const RobotState &meas) {
    RobotState rs;
    return rs;
}

