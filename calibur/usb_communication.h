#ifndef __CALIBUR_USB_COMMUNICATION_H__
#define __CALIBUR_USB_COMMUNICATION_H__

#include <string>
#include <cstdint>

namespace calibur {

    class USBCommunication {
        public:
        USBCommunication(const std::string& device_path);
        ~USBCommunication();
        
        bool open();
        bool close();
        bool isOpen() const;
        
        // Send yaw and pitch
        bool sendData(float yaw, float pitch, bool is_fire);
        
        // Configuration
        bool configure(int baudrate = 115200);

        private:
            std::string device_path_;
            int fd_;
            bool is_open_;
    };  
}

#endif