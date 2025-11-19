#include "types.hpp"
#include "workers.hpp"a


IMUWorker::IMUWorker(SharedLatest &shared,
            std::atomic<bool> &stop_flag)
    : shared_(shared), stop_(stop_flag) {}

void IMUWorker::operator()() {
    while (!stop_.load(std::memory_order_relaxed)) {
        IMUState imu;
        if (!read_imu_stub(imu)) {
            continue;
        }
        imu.timestamp = Clock::now();
        auto ptr = std::make_shared<IMUState>(std::move(imu));
        std::atomic_store(&shared_.imu, ptr);
        shared_.imu_ver.fetch_add(1, std::memory_order_relaxed);
    }
}

bool IMUWorker::read_imu_stub(IMUState &imu) {
    //TODO: replace with real IMU read
    imu.euler_angle = {0.0f, 0.0f, 0.0f};
    imu.time = 0.0f;
    return true;
}

