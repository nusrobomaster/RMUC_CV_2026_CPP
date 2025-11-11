#include "calibur/usb_communication.h"
#include "calibur/log.h"
#include <unistd.h>

int main() {
    calibur::Logger::ptr logger = CALIBUR_LOG_ROOT();
    
    CALIBUR_LOG_INFO(logger) << "=== USB Communication Test (yaw, pitch, is_fire) ===";
    
    calibur::USBCommunication usb("/dev/ttyUSB0");
    
    if (!usb.open()) {
        CALIBUR_LOG_FATAL(logger) << "Failed to open USB device - exiting";
        return 1;
    }
    
    CALIBUR_LOG_INFO(logger) << "USB device opened successfully";
    
    float yaw = 0.0f;
    float pitch = 0.0f;
    bool is_fire = false;
    int counter = 0;
    
    while (true) {
        yaw += 1.0f;
        pitch += 0.5f;
        
        if (yaw > 360.0f) yaw = 0.0f;
        if (pitch > 90.0f) pitch = -90.0f;
        
        counter++;
        if (counter % 5 == 0) {
            is_fire = !is_fire;
        }
        
        // Send
        if (usb.sendData(yaw, pitch, is_fire)) {
            CALIBUR_LOG_INFO(logger) << "Sent: yaw=" << yaw 
                                     << ", pitch=" << pitch 
                                     << ", is_fire=" << (is_fire ? "TRUE" : "FALSE");
        } else {
            CALIBUR_LOG_ERROR(logger) << "Failed to send data";
        }
        
        sleep(1); // Send every second
    }
    usb.close();
    return 0;
}