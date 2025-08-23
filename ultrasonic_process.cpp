#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <cmath>

#include "shared_memory.h"

// GPIO Pin definitions for ultrasonic sensor (BCM numbering)
#define ULTRASONIC_TRIG_PIN     24  // GPIO24 - Trigger pin
#define ULTRASONIC_ECHO_PIN     25  // GPIO25 - Echo pin

// Global variables for system control
volatile bool running = true;
SharedMemoryManager* g_shm = nullptr;

// Signal handler for clean shutdown
void signalHandler(int signal __attribute__((unused))) {
    running = false;
    std::cout << "\nShutting down ultrasonic sensor..." << std::endl;
    if (g_shm && g_shm->getData()) {
        g_shm->getData()->ultrasonic_active = false;
    }
}

// GPIO Pin management class (adapted from motor_process.cpp)
class GPIOPin {
private:
    int pin;
    int actualPin;
    
    bool writeToFile(const std::string& path, const std::string& value) {
        std::ofstream file(path);
        if (!file.is_open()) {
            std::cerr << "Failed to open file for writing: " << path << std::endl;
            return false;
        }
        file << value;
        file.close();
        return true;
    }
    
    std::string readFromFile(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            std::cerr << "Failed to open file for reading: " << path << std::endl;
            return "";
        }
        std::string value;
        file >> value;
        file.close();
        return value;
    }
    
    int detectGPIOOffset() {
        // Check for Pi 5 RP1 GPIO chip (gpiochip571)
        std::ifstream baseFile("/sys/class/gpio/gpiochip571/base");
        if (baseFile.is_open()) {
            int base;
            baseFile >> base;
            baseFile.close();
            std::cout << "Detected Pi 5 RP1 GPIO chip with base " << base << std::endl;
            return base;
        }
        
        // Check for Pi Zero 2W and other Pi models
        std::ifstream fallbackFile("/sys/class/gpio/gpiochip512/base");
        if (fallbackFile.is_open()) {
            int base;
            fallbackFile >> base;
            fallbackFile.close();
            std::cout << "Detected Pi with GPIO chip base " << base << std::endl;
            return base;
        }
        
        // Fallback for older Pi models
        std::cout << "Using direct BCM GPIO numbering (fallback)" << std::endl;
        return 0;
    }
    
public:
    GPIOPin(int pinNumber) : pin(pinNumber) {
        int offset = detectGPIOOffset();
        actualPin = pin + offset;
        std::cout << "GPIO" << pin << " mapped to sysfs GPIO" << actualPin << std::endl;
    }
    
    bool exportPin() {
        // Check if pin is already exported
        std::string gpioPath = "/sys/class/gpio/gpio" + std::to_string(actualPin);
        struct stat buffer;
        if (stat(gpioPath.c_str(), &buffer) == 0) {
            std::cout << "Pin " << pin << " (GPIO" << actualPin << ") already exported" << std::endl;
            return true;
        }
        
        std::cout << "Exporting pin " << pin << " (GPIO" << actualPin << ")..." << std::endl;
        std::ofstream exportFile("/sys/class/gpio/export");
        if (!exportFile.is_open()) {
            std::cerr << "Failed to open export file. Check permissions." << std::endl;
            return false;
        }
        
        exportFile << actualPin;
        exportFile.close();
        
        // Wait for the GPIO directory to be created
        for (int i = 0; i < 50; i++) {
            if (stat(gpioPath.c_str(), &buffer) == 0) {
                std::cout << "GPIO directory created after " << (i * 100) << "ms" << std::endl;
                break;
            }
            usleep(100000); // 100ms
        }
        
        if (stat(gpioPath.c_str(), &buffer) != 0) {
            std::cerr << "GPIO directory was not created: " << gpioPath << std::endl;
            return false;
        }
        
        // Wait for direction file
        std::string directionPath = gpioPath + "/direction";
        for (int i = 0; i < 50; i++) {
            if (stat(directionPath.c_str(), &buffer) == 0) {
                std::cout << "Direction file available after " << (i * 100) << "ms" << std::endl;
                return true;
            }
            usleep(100000); // 100ms
        }
        
        std::cerr << "Timeout waiting for direction file: " << directionPath << std::endl;
        return false;
    }
    
    bool unexportPin() {
        return writeToFile("/sys/class/gpio/unexport", std::to_string(actualPin));
    }
    
    bool setDirection(const std::string& dir) {
        std::string path = "/sys/class/gpio/gpio" + std::to_string(actualPin) + "/direction";
        std::cout << "Setting direction for pin " << pin << " (GPIO" << actualPin << ") to " << dir << std::endl;
        
        usleep(50000); // 50ms delay for stability
        bool result = writeToFile(path, dir);
        
        if (!result) {
            std::cerr << "Failed to set direction for pin " << pin << std::endl;
            // Retry once after a longer delay
            usleep(500000); // 500ms
            result = writeToFile(path, dir);
            if (result) {
                std::cout << "Direction set successfully on retry" << std::endl;
            }
        } else {
            std::cout << "Direction set successfully" << std::endl;
        }
        return result;
    }
    
    bool setValue(int value) {
        std::string path = "/sys/class/gpio/gpio" + std::to_string(actualPin) + "/value";
        std::ofstream file(path);
        if (!file.is_open()) {
            // Don't print error during shutdown - GPIO may be already unexported
            return false;
        }
        file << value;
        file.close();
        return !file.fail();
    }
    
    int getValue() {
        std::string path = "/sys/class/gpio/gpio" + std::to_string(actualPin) + "/value";
        std::string value = readFromFile(path);
        return !value.empty() ? std::stoi(value) : -1;
    }
    
    int getPin() const { return pin; }
    int getActualPin() const { return actualPin; }
};

// Ultrasonic Sensor Controller class
class UltrasonicSensor {
private:
    GPIOPin trigPin;
    GPIOPin echoPin;
    bool initialized;
    
    // Moving average filter for noise reduction
    static const int FILTER_SIZE = 5;
    float distances[FILTER_SIZE];
    int filterIndex;
    bool filterFull;
    
    float calculateFilteredDistance() {
        if (!filterFull && filterIndex < FILTER_SIZE) {
            // Not enough samples yet, return current reading
            return distances[filterIndex > 0 ? filterIndex - 1 : 0];
        }
        
        float sum = 0;
        int count = filterFull ? FILTER_SIZE : filterIndex;
        for (int i = 0; i < count; i++) {
            sum += distances[i];
        }
        return sum / count;
    }
    
    void addDistanceToFilter(float distance) {
        distances[filterIndex] = distance;
        filterIndex = (filterIndex + 1) % FILTER_SIZE;
        if (filterIndex == 0) {
            filterFull = true;
        }
    }
    
public:
    UltrasonicSensor() : 
        trigPin(ULTRASONIC_TRIG_PIN),
        echoPin(ULTRASONIC_ECHO_PIN),
        initialized(false),
        filterIndex(0),
        filterFull(false) {
        
        // Initialize filter array
        for (int i = 0; i < FILTER_SIZE; i++) {
            distances[i] = 999.0f; // Max distance
        }
    }
    
    bool initialize() {
        std::cout << "Initializing ultrasonic sensor..." << std::endl;
        std::cout << "Trigger pin: GPIO" << ULTRASONIC_TRIG_PIN << std::endl;
        std::cout << "Echo pin: GPIO" << ULTRASONIC_ECHO_PIN << std::endl;
        
        // Export and configure GPIO pins
        if (!trigPin.exportPin() || !trigPin.setDirection("out")) {
            std::cerr << "Failed to setup trigger pin" << std::endl;
            return false;
        }
        
        if (!echoPin.exportPin() || !echoPin.setDirection("in")) {
            std::cerr << "Failed to setup echo pin" << std::endl;
            return false;
        }
        
        // Initialize trigger pin to low
        trigPin.setValue(0);
        usleep(100000); // 100ms initial delay
        
        initialized = true;
        std::cout << "Ultrasonic sensor initialized successfully!" << std::endl;
        return true;
    }
    
    float measureDistance() {
        if (!initialized) {
            std::cerr << "Ultrasonic sensor not initialized!" << std::endl;
            return -1.0f;
        }
        
        // Send trigger pulse (10us high pulse)
        trigPin.setValue(0);
        usleep(2); // 2us low
        trigPin.setValue(1);
        usleep(10); // 10us high
        trigPin.setValue(0);
        
        // Wait for echo to go high (start of pulse)
        auto start_wait = std::chrono::high_resolution_clock::now();
        while (echoPin.getValue() == 0) {
            auto now = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_wait).count();
            if (duration > 15) { // 15ms timeout (increased for reliability)
                // Don't print error every time - this is normal occasionally
                return -1.0f;
            }
        }
        
        // Record time when echo goes high
        auto pulse_start = std::chrono::high_resolution_clock::now();
        
        // Wait for echo to go low (end of pulse)
        while (echoPin.getValue() == 1) {
            auto now = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - pulse_start).count();
            if (duration > 35) { // 35ms timeout (increased for reliability, corresponds to ~600cm max range)
                // Don't print error every time - this is normal occasionally
                return -1.0f;
            }
        }
        
        // Record time when echo goes low
        auto pulse_end = std::chrono::high_resolution_clock::now();
        
        // Calculate distance
        auto pulse_duration = std::chrono::duration_cast<std::chrono::microseconds>(pulse_end - pulse_start).count();
        
        // Distance = (pulse_duration * speed_of_sound) / 2
        // Speed of sound = 343 m/s = 0.0343 cm/us
        // Distance in cm = (pulse_duration_us * 0.0343) / 2
        float distance = (pulse_duration * 0.0343f) / 2.0f;
        
        // Validate reading (typical HC-SR04 range is 2-400cm)
        if (distance < 2.0f || distance > 400.0f) {
            // Don't print error message for out of range readings - this is normal
            return -1.0f;
        }
        
        // Add to moving average filter
        addDistanceToFilter(distance);
        
        return calculateFilteredDistance();
    }
    
    void cleanup() {
        if (initialized) {
            // Try to set trigger pin low, but don't error if it fails (may be already unexported)
            trigPin.setValue(0);
            // Unexport pins - don't check return values as they may already be unexported
            trigPin.unexportPin();
            echoPin.unexportPin();
        }
    }
    
    ~UltrasonicSensor() {
        cleanup();
    }
};

int main() {
    // Set up signal handler for clean shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::cout << "Starting Wall-E Ultrasonic Sensor Process..." << std::endl;
    
    // Initialize shared memory (not as creator, control_interface should create it)
    SharedMemoryManager shm(false);
    g_shm = &shm;
    
    // Wait for shared memory to be created by another process
    int retry_count = 0;
    while (!shm.initialize() && retry_count < 100) {
        std::cout << "Waiting for shared memory to be created..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        retry_count++;
    }
    
    if (retry_count >= 100) {
        std::cerr << "Failed to connect to shared memory. Is the control system running?" << std::endl;
        return 1;
    }
    
    BalanceData* data = shm.getData();
    data->ultrasonic_active = true;
    data->ultrasonic_enabled = true;
    
    // Initialize ultrasonic sensor
    UltrasonicSensor sensor;
    if (!sensor.initialize()) {
        std::cerr << "Failed to initialize ultrasonic sensor" << std::endl;
        data->ultrasonic_active = false;
        return 1;
    }
    
    std::cout << "Ultrasonic sensor process running..." << std::endl;
    std::cout << "Minimum safe distance: " << data->ultrasonic_min_distance << " cm" << std::endl;
    
    // Main measurement loop - 50ms cycle time (20Hz) for responsive obstacle detection
    auto next_cycle = std::chrono::steady_clock::now();
    const auto cycle_duration = std::chrono::milliseconds(50);
    
    int measurement_count = 0;
    int error_count = 0;
    
    while (running) {
        // Check if ultrasonic is enabled
        if (!data->ultrasonic_enabled) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        // Measure distance
        float distance = sensor.measureDistance();
        
        if (distance > 0) {
            // Valid measurement
            data->ultrasonic_distance_cm = distance;
            data->ultrasonic_timestamp = getCurrentTimeMicros();
            
            // Check for obstacle detection
            bool previous_obstacle = data->obstacle_detected;
            data->obstacle_detected = (distance <= data->ultrasonic_min_distance);
            
            // Log obstacle state changes
            if (data->obstacle_detected && !previous_obstacle) {
                std::cout << "OBSTACLE DETECTED! Distance: " << distance << " cm (min: " 
                          << data->ultrasonic_min_distance << " cm)" << std::endl;
            } else if (!data->obstacle_detected && previous_obstacle) {
                std::cout << "Obstacle cleared. Distance: " << distance << " cm" << std::endl;
            }
            
            measurement_count++;
            
            // Debug output every 2 seconds (40 cycles at 20Hz)
            if (measurement_count % 40 == 0) {
                std::cout << "Distance: " << distance << " cm | " 
                          << "Obstacle: " << (data->obstacle_detected ? "YES" : "NO") << " | "
                          << "Errors: " << error_count << std::endl;
            }
        } else {
            // Invalid measurement - increment error count but don't spam console
            error_count++;
            // Only report errors periodically to avoid spam
            if (error_count % 50 == 0) {
                std::cerr << "Ultrasonic measurement errors: " << error_count << " (timeouts/out-of-range readings)" << std::endl;
            }
        }
        
        // Precise timing control
        next_cycle += cycle_duration;
        std::this_thread::sleep_until(next_cycle);
        
        // Check if we're falling behind
        auto now = std::chrono::steady_clock::now();
        if (now > next_cycle) {
            next_cycle = now;
        }
    }
    
    // Cleanup
    data->ultrasonic_active = false;
    data->obstacle_detected = false;
    sensor.cleanup();
    
    std::cout << "Ultrasonic sensor process shutdown complete." << std::endl;
    std::cout << "Total measurements: " << measurement_count << std::endl;
    std::cout << "Total errors: " << error_count << std::endl;
    
    return 0;
}
