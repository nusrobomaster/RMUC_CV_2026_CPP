#include "types.hpp"
#include <thread>
#include "workers.hpp"
#include "usb_communication.h"
#include "protocol_data.hpp"
#include "../log.h"

USBWorker::USBWorker(SharedLatest &shared,
            SharedScalars &scalars,
            std::atomic<bool> &stop_flag,
            std::shared_ptr<calibur::USBCommunication> usb_comm)
    : shared_(shared), scalars_(scalars), stop_(stop_flag), last_pred_ver_(0), usb_comm_(std::move(usb_comm)) {}

void USBWorker::operator()() {
    if (!usb_comm_->open()) {
        std::cerr << "[USBWorker] Failed to open USB device!" << std::endl;
        return;
    }
    usb_comm_->configure(115200);

    // 1. Initialize a "previous data" packet with zeros
    Protocol::AimbotData last_pkt{0.0f, 0.0f, 0.0f};
    
    const auto loop_interval = std::chrono::milliseconds(10); // 100Hz

    while (!stop_.load(std::memory_order_relaxed)) {
        auto start_time = std::chrono::steady_clock::now();

        process_usb_rx();

        // 2. Try to get new data from the PredictionWorker
        auto pred_ptr = std::atomic_load(&shared_.prediction_out);

        if (pred_ptr) {
            // New data exists: Update our "previous data" buffer
            last_pkt.yaw   = pred_ptr->yaw + -0.05;
            last_pkt.pitch = pred_ptr->pitch + 0.05;
            last_pkt.fire  = static_cast<float>(pred_ptr->fire);
        } 
        // else: we don't update last_pkt, keeping the previous values

        // 3. ALWAYS send the current state of last_pkt
        usb_comm_->sendData(Protocol::Type::aimbot, &last_pkt, sizeof(last_pkt));

        // 4. Maintain constant 100Hz frequency
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed = end_time - start_time;
        if (elapsed < loop_interval) {
            std::this_thread::sleep_for(loop_interval - elapsed);
        }
    }
}

void USBWorker::process_usb_rx() {
    // parse incoming packets if you later implement MCU->Jetson
}

void USBWorker::usb_send_tx(const PredictionOut &out) {
    // If you want to send real prediction:
    // Protocol::AimbotData pkt{ .yaw = out.yaw, .pitch = out.pitch, .fire = out.fire ? 1.0f : 0.0f };
    // usb_comm_->sendData(Protocol::Type::aimbot, &pkt, sizeof(pkt));
}