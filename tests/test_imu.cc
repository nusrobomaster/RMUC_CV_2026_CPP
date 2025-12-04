/*
 * gimbal_calibrate_standalone.cpp
 * 
 * Fully standalone gimbal calibration tool.
 * Reads IMU directly, no external dependencies needed.
 * 
 * Compile:
 *   g++ -std=c++17 gimbal_calibrate_standalone.cpp -o gimbal_calibrate -lpthread
 * 
 * Run:
 *   ./gimbal_calibrate
 */

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <atomic>
#include <cmath>
#include <vector>
#include <algorithm>
#include <memory>
#include <fstream>
#include <sstream>

// ============================================================================
// MINIMAL TYPE DEFINITIONS (no external dependencies)
// ============================================================================

struct IMUData {
    float roll;   // degrees
    float pitch;  // degrees
    float yaw;    // degrees
    
    IMUData() : roll(0), pitch(0), yaw(0) {}
};

// ============================================================================
// IMU READER (Adapt this to YOUR specific IMU interface)
// ============================================================================

class IMUReader {
private:
    std::atomic<bool> running_;
    std::atomic<float> current_pitch_;
    std::atomic<float> current_yaw_;
    std::atomic<bool> data_available_;
    
public:
    IMUReader() 
        : running_(false), 
          current_pitch_(0.0f), 
          current_yaw_(0.0f),
          data_available_(false) 
    {}
    
    bool start() {
        // TODO: Initialize your IMU hardware here
        // This is a placeholder - replace with your actual IMU initialization
        
        std::cout << "Initializing IMU..." << std::flush;
        
        // Simulate IMU initialization
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // TODO: Replace this with your actual IMU test
        // For example:
        // if (!imu_device.open()) return false;
        // if (!imu_device.configure()) return false;
        
        std::cout << " Done!\n";
        
        running_.store(true);
        return true;
    }
    
    void stop() {
        running_.store(false);
        // TODO: Cleanup your IMU hardware here
    }
    
    bool read_current_values(float &pitch_rad, float &yaw_rad) {
        if (!running_.load()) {
            return false;
        }
        
        // TODO: Replace this with your actual IMU reading code
        // This is where you read from your IMU device
        
        // Example for different IMU types:
        
        // === OPTION 1: Reading from shared memory (your current system) ===
        // Uncomment and adapt this if you're using your existing SharedLatest:
        /*
        extern SharedLatest global_shared;  // Declare your global shared
        std::shared_ptr<IMUState> imu_ptr = std::atomic_load(&global_shared.imu);
        if (!imu_ptr || imu_ptr->euler_angle.size() < 3) {
            return false;
        }
        float pitch_deg = imu_ptr->euler_angle[1];
        float yaw_deg = imu_ptr->euler_angle[2];
        pitch_rad = pitch_deg * M_PI / 180.0f;
        yaw_rad = yaw_deg * M_PI / 180.0f;
        */
        
        // === OPTION 2: Reading from serial port ===
        /*
        std::string line;
        if (serial_port.readline(line)) {
            // Parse line like: "PITCH=-12.5,YAW=45.2"
            // Extract pitch and yaw, convert to radians
            pitch_rad = extracted_pitch * M_PI / 180.0f;
            yaw_rad = extracted_yaw * M_PI / 180.0f;
        }
        */
        
        // === OPTION 3: Simulation for testing ===
        // Remove this once you implement real IMU reading!
        static float sim_pitch = 0.0f;
        static float sim_yaw = 0.0f;
        static int counter = 0;
        
        // Simulate some movement
        sim_pitch = 0.3f * std::sin(counter * 0.1f);
        sim_yaw = 0.5f * std::cos(counter * 0.05f);
        counter++;
        
        pitch_rad = sim_pitch;
        yaw_rad = sim_yaw;
        
        // Mark data as available
        data_available_.store(true);
        current_pitch_.store(pitch_rad);
        current_yaw_.store(yaw_rad);
        
        return true;
    }
    
    bool is_data_available() const {
        return data_available_.load();
    }
};

// ============================================================================
// CALIBRATION TOOL
// ============================================================================

class GimbalCalibrator {
private:
    IMUReader imu_;
    float pitch_min_;
    float pitch_max_;
    
public:
    GimbalCalibrator() 
        : pitch_min_(999.0f), 
          pitch_max_(-999.0f) 
    {}
    
    void run() {
        print_header();
        
        // Start IMU
        if (!imu_.start()) {
            std::cout << "❌ ERROR: Failed to initialize IMU!\n";
            std::cout << "\nPlease check:\n";
            std::cout << "  - IMU is connected and powered\n";
            std::cout << "  - You have permission to access the device\n";
            std::cout << "  - IMU driver is properly configured\n\n";
            
            std::cout << "TODO: Edit IMUReader::start() in this file to match your IMU.\n";
            return;
        }
        
        // Test connection
        if (!test_connection()) {
            std::cout << "❌ ERROR: Cannot read data from IMU!\n\n";
            std::cout << "TODO: Edit IMUReader::read_current_values() to match your IMU.\n";
            imu_.stop();
            return;
        }
        
        std::cout << "✅ IMU connection OK!\n\n";
        
        // Run calibration steps
        calibrate_min();
        calibrate_max();
        show_results();
        export_results();
        
        // Cleanup
        imu_.stop();
    }
    
private:
    void print_header() {
        std::cout << "\n";
        std::cout << "╔════════════════════════════════════════════════════════╗\n";
        std::cout << "║     GIMBAL PITCH CALIBRATION TOOL v2.0 (Standalone)   ║\n";
        std::cout << "╚════════════════════════════════════════════════════════╝\n";
        std::cout << "\n";
        std::cout << "This tool will measure your gimbal's physical pitch limits.\n";
        std::cout << "You'll manually move the gimbal to extreme positions.\n\n";
    }
    
    bool test_connection() {
        std::cout << "Testing IMU connection";
        
        float pitch, yaw;
        for (int i = 0; i < 30; i++) {
            if (imu_.read_current_values(pitch, yaw)) {
                std::cout << " ";
                return true;
            }
            std::cout << "." << std::flush;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::cout << "\n";
        return false;
    }
    
    void calibrate_min() {
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
        std::cout << "STEP 1 of 2: MINIMUM PITCH (Looking DOWN)\n";
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n";
        
        std::cout << "Instructions:\n";
        std::cout << "  1. MANUALLY tilt gimbal DOWN as far as it can go\n";
        std::cout << "  2. Make sure it hits the PHYSICAL STOP (hard limit)\n";
        std::cout << "  3. The gimbal should NOT be able to tilt down further\n";
        std::cout << "  4. Hold it STEADY at that position\n\n";
        
        // Show live values for 5 seconds
        show_live_values(5);
        
        std::cout << "\n";
        std::cout << "Is the gimbal at its LOWEST position? Press ENTER...";
        std::cin.get();
        
        // Record samples
        std::vector<float> samples = record_pitch_samples(50);
        pitch_min_ = get_median(samples);
        
        std::cout << "\n✅ MINIMUM PITCH: " << std::fixed << std::setprecision(4) 
                  << pitch_min_ << " rad (" 
                  << (pitch_min_ * 180.0f / M_PI) << "°)\n\n";
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    
    void calibrate_max() {
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
        std::cout << "STEP 2 of 2: MAXIMUM PITCH (Looking UP)\n";
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n";
        
        std::cout << "Instructions:\n";
        std::cout << "  1. MANUALLY tilt gimbal UP as far as it can go\n";
        std::cout << "  2. Make sure it hits the PHYSICAL STOP (hard limit)\n";
        std::cout << "  3. The gimbal should NOT be able to tilt up further\n";
        std::cout << "  4. Hold it STEADY at that position\n\n";
        
        // Show live values for 5 seconds
        show_live_values(5);
        
        std::cout << "\n";
        std::cout << "Is the gimbal at its HIGHEST position? Press ENTER...";
        std::cin.get();
        
        // Record samples
        std::vector<float> samples = record_pitch_samples(50);
        pitch_max_ = get_median(samples);
        
        std::cout << "\n✅ MAXIMUM PITCH: " << std::fixed << std::setprecision(4) 
                  << pitch_max_ << " rad (" 
                  << (pitch_max_ * 180.0f / M_PI) << "°)\n\n";
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    
    void show_live_values(int seconds) {
        std::cout << "Current values (move gimbal to position):\n";
        
        float pitch, yaw;
        int iterations = seconds * 10;  // 10 Hz update rate
        
        for (int i = 0; i < iterations; i++) {
            if (imu_.read_current_values(pitch, yaw)) {
                std::cout << "\rPitch: " << std::setw(7) << std::fixed << std::setprecision(4) 
                          << pitch << " rad (" 
                          << std::setw(7) << std::setprecision(2) 
                          << (pitch * 180.0f / M_PI) << "°)   " << std::flush;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    std::vector<float> record_pitch_samples(int count) {
        std::vector<float> samples;
        samples.reserve(count);
        
        std::cout << "Recording " << count << " samples";
        
        float pitch, yaw;
        for (int i = 0; i < count; i++) {
            if (imu_.read_current_values(pitch, yaw)) {
                samples.push_back(pitch);
            }
            std::cout << "." << std::flush;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        
        std::cout << " Done!";
        return samples;
    }
    
    float get_median(std::vector<float> samples) {
        if (samples.empty()) return 0.0f;
        
        std::sort(samples.begin(), samples.end());
        size_t n = samples.size();
        
        if (n % 2 == 0) {
            return (samples[n/2 - 1] + samples[n/2]) / 2.0f;
        } else {
            return samples[n/2];
        }
    }
    
    void show_results() {
        std::cout << "\n\n";
        std::cout << "╔════════════════════════════════════════════════════════╗\n";
        std::cout << "║              CALIBRATION RESULTS                       ║\n";
        std::cout << "╚════════════════════════════════════════════════════════╝\n\n";
        
        if (pitch_min_ >= 900.0f || pitch_max_ <= -900.0f) {
            std::cout << "❌ ERROR: No valid data recorded!\n";
            return;
        }
        
        float range = pitch_max_ - pitch_min_;
        
        std::cout << "Measured pitch limits:\n";
        std::cout << "┌─────────────────────────────────────────────────────┐\n";
        std::cout << "│ MIN (down): " << std::setw(9) << std::fixed << std::setprecision(4)
                  << pitch_min_ << " rad = " 
                  << std::setw(7) << std::setprecision(2) 
                  << (pitch_min_ * 180.0f / M_PI) << "° │\n";
        std::cout << "│ MAX (up):   " << std::setw(9) << pitch_max_ << " rad = " 
                  << std::setw(7) << (pitch_max_ * 180.0f / M_PI) << "° │\n";
        std::cout << "│ RANGE:      " << std::setw(9) << range << " rad = " 
                  << std::setw(7) << (range * 180.0f / M_PI) << "° │\n";
        std::cout << "└─────────────────────────────────────────────────────┘\n\n";
        
        // Validation
        if (range < 0.1f) {
            std::cout << "⚠️  WARNING: Range is very small (" << (range * 180.0f / M_PI) << "°)\n";
            std::cout << "   Did you move the gimbal between measurements?\n\n";
        } else if (range > 3.5f) {
            std::cout << "⚠️  WARNING: Range is very large (" << (range * 180.0f / M_PI) << "°)\n";
            std::cout << "   Please verify the measurements are correct.\n\n";
        } else {
            std::cout << "✅ Measurements look good!\n\n";
        }
    }
    
    void export_results() {
        if (pitch_min_ >= 900.0f || pitch_max_ <= -900.0f) {
            return;
        }
        
        // Add 3° safety margin
        const float margin = 0.0524f;  // ~3 degrees
        float safe_min = pitch_min_ + margin;
        float safe_max = pitch_max_ - margin;
        
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
        std::cout << "  COPY THESE LINES TO YOUR CODE\n";
        std::cout << "  (Add to types.hpp or config.h)\n";
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n";
        
        std::cout << "// Gimbal physical limits (calibrated)\n";
        std::cout << "constexpr float GIMBAL_PITCH_MIN = " << std::setprecision(5) 
                  << safe_min << "f;  // " << std::setprecision(2)
                  << (safe_min * 180.0f / M_PI) << "° (down)\n";
        std::cout << "constexpr float GIMBAL_PITCH_MAX = " << std::setprecision(5)
                  << safe_max << "f;  // " << std::setprecision(2)
                  << (safe_max * 180.0f / M_PI) << "° (up)\n\n";
        
        std::cout << "// Yaw limits (360° rotation)\n";
        std::cout << "constexpr float GIMBAL_YAW_MIN = -3.14159f;  // -180°\n";
        std::cout << "constexpr float GIMBAL_YAW_MAX =  3.14159f;  // +180°\n";
        std::cout << "constexpr bool GIMBAL_HAS_YAW_LIMITS = false;\n\n";
        
        std::cout << "// Safety margin\n";
        std::cout << "constexpr float GIMBAL_SAFETY_MARGIN = 0.0524f;  // ~3°\n\n";
        
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n";
        
        // Save to file
        save_to_file(safe_min, safe_max);
    }
    
    void save_to_file(float safe_min, float safe_max) {
        const char* filename = "gimbal_limits_config.h";
        std::ofstream file(filename);
        
        if (!file.is_open()) {
            std::cout << "⚠️  Could not create output file\n";
            return;
        }
        
        file << "// Gimbal calibration results\n";
        file << "// Generated by gimbal_calibrate tool\n\n";
        file << "#ifndef GIMBAL_LIMITS_H\n";
        file << "#define GIMBAL_LIMITS_H\n\n";
        file << std::fixed << std::setprecision(5);
        file << "constexpr float GIMBAL_PITCH_MIN = " << safe_min << "f;  // " 
             << (safe_min * 180.0f / M_PI) << "° (down)\n";
        file << "constexpr float GIMBAL_PITCH_MAX = " << safe_max << "f;  // " 
             << (safe_max * 180.0f / M_PI) << "° (up)\n";
        file << "constexpr float GIMBAL_YAW_MIN = -3.14159f;  // -180°\n";
        file << "constexpr float GIMBAL_YAW_MAX =  3.14159f;  // +180°\n";
        file << "constexpr bool GIMBAL_HAS_YAW_LIMITS = false;\n";
        file << "constexpr float GIMBAL_SAFETY_MARGIN = 0.0524f;  // ~3°\n\n";
        file << "#endif // GIMBAL_LIMITS_H\n";
        
        file.close();
        
        std::cout << "✅ Saved to: " << filename << "\n\n";
    }
};

// ============================================================================
// MAIN FUNCTION
// ============================================================================

int main(int argc, char** argv) {
    std::cout << "\n🎯 Gimbal Calibration Tool - Starting...\n\n";
    
    // Check if user wants help
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n\n";
            std::cout << "Options:\n";
            std::cout << "  --help, -h    Show this help message\n\n";
            std::cout << "This tool will guide you through calibrating your gimbal's\n";
            std::cout << "pitch limits by moving it to extreme positions.\n\n";
            return 0;
        }
    }
    
    std::cout << "⚠️  IMPORTANT: Before running, you must edit this file!\n\n";
    std::cout << "Edit the IMUReader class to match YOUR IMU interface:\n";
    std::cout << "  1. Open: " << argv[0] << ".cpp\n";
    std::cout << "  2. Find: IMUReader::read_current_values()\n";
    std::cout << "  3. Replace the simulation code with your actual IMU reading\n\n";
    std::cout << "Continue anyway? (y/n): ";
    
    char choice;
    std::cin >> choice;
    std::cin.ignore(10000, '\n');
    
    if (choice != 'y' && choice != 'Y') {
        std::cout << "\nExiting. Please edit the code first!\n";
        return 0;
    }
    
    // Run calibration
    GimbalCalibrator calibrator;
    calibrator.run();
    
    std::cout << "✅ Calibration complete!\n";
    std::cout << "\nPress ENTER to exit...";
    std::cin.get();
    
    return 0;
}