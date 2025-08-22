#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <sys/stat.h>
#include <poll.h>
#include <cmath>

#include "shared_memory.h"

// GPIO Pin definitions (BCM numbering) - matching your motor controller
#define LEFT_MOTOR_PWM      12
#define RIGHT_MOTOR_PWM     13  
#define LEFT_MOTOR_DIR      26
#define RIGHT_MOTOR_DIR     21
#define MOTOR_ENABLE        20
#define LEFT_ENCODER        17
#define RIGHT_ENCODER       27

// Global variables for encoder counting and system control
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile bool running = true;
SharedMemoryManager* g_shm = nullptr;

// Signal handler for clean shutdown
void signalHandler(int signal __attribute__((unused))) {
    running = false;
    std::cout << "\nShutting down motor controller..." << std::endl;
    if (g_shm && g_shm->getData()) {
        g_shm->getData()->motor_active = false;
        g_shm->getData()->emergency_stop = true;
    }
}

// GPIO Pin management class (simplified from your motorCtrl.cpp)
class GPIOPin {
private:
    int pin;
    int actualPin;
    
    bool writeToFile(const std::string& path, const std::string& value) {
        std::ofstream file(path);
        if (!file.is_open()) return false;
        file << value;
        file.close();
        return true;
    }
    
    int detectGPIOOffset() {
        // Check for Pi 5 RP1 GPIO chip (gpiochip571)
        std::ifstream baseFile("/sys/class/gpio/gpiochip571/base");
        if (baseFile.is_open()) {
            int base;
            baseFile >> base;
            baseFile.close();
            return base;
        }
        
        // Fallback for other Pi models
        std::ifstream fallbackFile("/sys/class/gpio/gpiochip512/base");
        if (fallbackFile.is_open()) {
            int base;
            fallbackFile >> base;
            fallbackFile.close();
            return base;
        }
        
        return 0; // Direct BCM numbering
    }
    
public:
    GPIOPin(int pinNumber) : pin(pinNumber) {
        int offset = detectGPIOOffset();
        actualPin = pin + offset;
    }
    
    bool exportPin() {
        std::string gpioPath = "/sys/class/gpio/gpio" + std::to_string(actualPin);
        struct stat buffer;
        if (stat(gpioPath.c_str(), &buffer) == 0) return true;
        
        std::ofstream exportFile("/sys/class/gpio/export");
        if (!exportFile.is_open()) return false;
        exportFile << actualPin;
        exportFile.close();
        
        // Wait for directory creation
        for (int i = 0; i < 50; i++) {
            if (stat(gpioPath.c_str(), &buffer) == 0) break;
            usleep(100000);
        }
        return stat(gpioPath.c_str(), &buffer) == 0;
    }
    
    bool setDirection(const std::string& dir) {
        std::string path = "/sys/class/gpio/gpio" + std::to_string(actualPin) + "/direction";
        usleep(50000);
        return writeToFile(path, dir);
    }
    
    bool setValue(int value) {
        std::string path = "/sys/class/gpio/gpio" + std::to_string(actualPin) + "/value";
        return writeToFile(path, std::to_string(value));
    }
    
    bool setEdge(const std::string& edge) {
        std::string path = "/sys/class/gpio/gpio" + std::to_string(actualPin) + "/edge";
        return writeToFile(path, edge);
    }
    
    int getActualPin() const { return actualPin; }
};

// PWM Pin management class (simplified from your motorCtrl.cpp)
class PWMPin {
private:
    int pin;
    int pwmChip;
    int pwmChannel;
    
    bool writeToFile(const std::string& path, const std::string& value) {
        std::ofstream file(path);
        if (!file.is_open()) return false;
        file << value;
        file.close();
        return true;
    }
    
public:
    PWMPin(int pinNumber) : pin(pinNumber) {
        if (pin == 12) {
            pwmChip = 0;
            pwmChannel = 0;
        } else if (pin == 13) {
            pwmChip = 0;
            pwmChannel = 1;
        } else {
            pwmChip = -1;
            pwmChannel = -1;
        }
    }
    
    bool initialize() {
        if (pwmChip == -1 || pwmChannel == -1) {
            std::cerr << "PWM pin " << pin << " not supported!" << std::endl;
            return false;
        }
        
        std::string basePath = "/sys/class/pwm/pwmchip" + std::to_string(pwmChip) + "/";
        
        // Export PWM channel
        std::cout << "Exporting PWM pin " << pin << " (chip " << pwmChip << ", channel " << pwmChannel << ")" << std::endl;
        if (!writeToFile(basePath + "export", std::to_string(pwmChannel))) {
            std::cerr << "Failed to export PWM channel!" << std::endl;
            return false;
        }
        usleep(100000);
        
        // Set period (50Hz = 20ms = 20000000 ns)
        std::string channelPath = basePath + "pwm" + std::to_string(pwmChannel) + "/";
        std::cout << "Setting PWM period for pin " << pin << std::endl;
        if (!writeToFile(channelPath + "period", "20000000")) {
            std::cerr << "Failed to set PWM period!" << std::endl;
            return false;
        }
        
        // Enable PWM
        std::cout << "Enabling PWM for pin " << pin << std::endl;
        if (!writeToFile(channelPath + "enable", "1")) {
            std::cerr << "Failed to enable PWM!" << std::endl;
            return false;
        }
        
        std::cout << "PWM pin " << pin << " initialized successfully!" << std::endl;
        return true;
    }
    
    bool setDutyCycle(int percentage) {
        if (pwmChip == -1 || pwmChannel == -1) return false;
        
        percentage = std::max(0, std::min(100, percentage));
        long dutyCycle = (20000000L * percentage) / 100;
        
        std::string channelPath = "/sys/class/pwm/pwmchip" + std::to_string(pwmChip) + 
                                  "/pwm" + std::to_string(pwmChannel) + "/";
        
        return writeToFile(channelPath + "duty_cycle", std::to_string(dutyCycle));
    }
    
    void cleanup() {
        if (pwmChip == -1 || pwmChannel == -1) return;
        
        std::string basePath = "/sys/class/pwm/pwmchip" + std::to_string(pwmChip) + "/";
        std::string channelPath = basePath + "pwm" + std::to_string(pwmChannel) + "/";
        
        writeToFile(channelPath + "enable", "0");
        writeToFile(basePath + "unexport", std::to_string(pwmChannel));
    }
};

// Balance Motor Controller class
class BalanceMotorController {
private:
    GPIOPin leftDirPin;
    GPIOPin rightDirPin;
    GPIOPin enablePin;
    GPIOPin leftEncoderPin;
    GPIOPin rightEncoderPin;
    PWMPin leftPwmPin;
    PWMPin rightPwmPin;
    
    std::thread leftEncoderThread;
    std::thread rightEncoderThread;
    
    bool initialized;
    
    // Balance control variables (PID state)
    int encoder_left_pulse_num_speed;
    int encoder_right_pulse_num_speed;
    double speed_control_output;
    double rotation_control_output;
    double speed_filter_old;
    
    void monitorEncoder(GPIOPin& encoderPin, volatile long& counter) {
        std::string path = "/sys/class/gpio/gpio" + std::to_string(encoderPin.getActualPin()) + "/value";
        int fd = open(path.c_str(), O_RDONLY);
        if (fd < 0) {
            std::cerr << "Failed to open encoder GPIO " << encoderPin.getActualPin() << std::endl;
            return;
        }
        
        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLPRI | POLLERR;
        
        int lastValue = -1;
        char buffer[8];
        
        // Read initial value
        lseek(fd, 0, SEEK_SET);
        int initialRead = read(fd, buffer, sizeof(buffer)-1);
        if (initialRead > 0) {
            buffer[initialRead] = '\0';
            lastValue = buffer[0] - '0';
        }
        
        while (running) {
            int result = poll(&pfd, 1, 100);  // Increased timeout for better detection
            if (result > 0) {
                if (pfd.revents & POLLPRI) {
                    lseek(fd, 0, SEEK_SET);
                    int bytesRead = read(fd, buffer, sizeof(buffer)-1);
                    if (bytesRead > 0) {
                        buffer[bytesRead] = '\0';
                        int currentValue = buffer[0] - '0';
                        
                        if (lastValue != -1 && currentValue != lastValue) {
                            counter++;
                        }
                        lastValue = currentValue;
                    }
                } else if (pfd.revents & POLLERR) {
                    std::cerr << "Poll error on encoder GPIO " << encoderPin.getActualPin() << std::endl;
                    break;
                }
            } else if (result < 0) {
                std::cerr << "Poll failed on encoder GPIO " << encoderPin.getActualPin() << std::endl;
                break;
            }
            // result == 0 means timeout, which is normal
        }
        
        close(fd);
    }
    
public:
    BalanceMotorController() : 
        leftDirPin(LEFT_MOTOR_DIR),
        rightDirPin(RIGHT_MOTOR_DIR),
        enablePin(MOTOR_ENABLE),
        leftEncoderPin(LEFT_ENCODER),
        rightEncoderPin(RIGHT_ENCODER),
        leftPwmPin(LEFT_MOTOR_PWM),
        rightPwmPin(RIGHT_MOTOR_PWM),
        initialized(false),
        encoder_left_pulse_num_speed(0),
        encoder_right_pulse_num_speed(0),
        speed_control_output(0.0),
        rotation_control_output(0.0),
        speed_filter_old(0.0) {}
    
    bool initialize() {
        // Setup GPIO pins
        if (!leftDirPin.exportPin() || !leftDirPin.setDirection("out")) return false;
        if (!rightDirPin.exportPin() || !rightDirPin.setDirection("out")) return false;
        if (!enablePin.exportPin() || !enablePin.setDirection("out")) return false;
        if (!leftEncoderPin.exportPin() || !leftEncoderPin.setDirection("in") || 
            !leftEncoderPin.setEdge("both")) return false;
        if (!rightEncoderPin.exportPin() || !rightEncoderPin.setDirection("in") || 
            !rightEncoderPin.setEdge("both")) return false;
        
        // Initialize PWM
        if (!leftPwmPin.initialize() || !rightPwmPin.initialize()) return false;
        
        // Enable motors
        enablePin.setValue(1);
        stopMotors();
        
        // Start encoder monitoring threads
        leftEncoderThread = std::thread(&BalanceMotorController::monitorEncoder, this, 
                                       std::ref(leftEncoderPin), std::ref(leftEncoderCount));
        rightEncoderThread = std::thread(&BalanceMotorController::monitorEncoder, this, 
                                        std::ref(rightEncoderPin), std::ref(rightEncoderCount));
        
        initialized = true;
        std::cout << "Balance motor controller initialized successfully!" << std::endl;
        return true;
    }
    
    void setMotor(bool isLeft, float pwm_value) {
        if (!initialized) return;
        
        // Convert PWM value from standard range (-255 to 255) to percentage (0-100)
        bool forward = pwm_value >= 0;
        int speed = (int)(std::abs(pwm_value) * 100.0f / 255.0f);
        speed = std::max(0, std::min(100, speed));
        
        if (isLeft) {
            leftDirPin.setValue(forward ? 0 : 1);  // Inverted for correct movement
            leftPwmPin.setDutyCycle(speed);
        } else {
            rightDirPin.setValue(forward ? 0 : 1); // Inverted for correct movement
            rightPwmPin.setDutyCycle(speed);
        }
    }
    
    void stopMotors() {
        if (!initialized) return;
        leftPwmPin.setDutyCycle(0);
        rightPwmPin.setDutyCycle(0);
    }
    
    void enableMotors(bool enable = true) {
        if (!initialized) return;
        enablePin.setValue(enable ? 1 : 0);
    }
    
    // Balance control function (PID implementation)
    void balanceControl(BalanceData* data) {
        // Update encoder counts (differential feedback)
        long leftCount = leftEncoderCount;
        long rightCount = rightEncoderCount;
        
        encoder_left_pulse_num_speed += data->pwm_left < 0 ? -leftCount : leftCount;
        encoder_right_pulse_num_speed += data->pwm_right < 0 ? -rightCount : rightCount;
        
        // Reset per-cycle counters
        leftEncoderCount = 0;
        rightEncoderCount = 0;
        
        // Balance control (PID for angle stabilization)
        double balance_control_output = KP_BALANCE * (data->kalman_angle - data->angle_zero) + 
                                       KD_BALANCE * (data->gyro_x - data->angular_velocity_zero);
        
        // Speed control (every 8 cycles = 40ms for stability)
        data->speed_control_period_count++;
        if (data->speed_control_period_count >= SPEED_CONTROL_PERIOD) {
            data->speed_control_period_count = 0;
            
            double car_speed = (encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
            encoder_left_pulse_num_speed = 0;
            encoder_right_pulse_num_speed = 0;
            
            data->speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
            speed_filter_old = data->speed_filter;
            
            data->car_speed_integral += data->speed_filter;
            data->car_speed_integral += -data->setting_car_speed;
            constrainValue(data->car_speed_integral, -3000.0, 3000.0);
            
            speed_control_output = -KP_SPEED * data->speed_filter - KI_SPEED * data->car_speed_integral;
            rotation_control_output = data->setting_turn_speed + KD_TURN * data->gyro_z;
        }
        
        // Calculate motor PWM values (balanced output)
        data->pwm_left = balance_control_output - speed_control_output - rotation_control_output;
        data->pwm_right = balance_control_output - speed_control_output + rotation_control_output;
        
        constrainValue(data->pwm_left, -255.0f, 255.0f);
        constrainValue(data->pwm_right, -255.0f, 255.0f);
        
        // Emergency stop logic (safety limits)
        if (data->motion_mode != START && data->motion_mode != STOP && 
            (data->kalman_angle < BALANCE_ANGLE_MIN || data->kalman_angle > BALANCE_ANGLE_MAX)) {
            data->motion_mode = STOP;
            data->emergency_stop = true;
            stopMotors();
            return;
        }
        
        // Motor control logic (state-based operation)
        if (data->motion_mode == STOP || data->emergency_stop || data->safety_override) {
            data->car_speed_integral = 0;
            data->setting_car_speed = 0;
            data->pwm_left = 0;
            data->pwm_right = 0;
            stopMotors();
        } else {
            // Apply motor control
            setMotor(true, data->pwm_left);   // Left motor
            setMotor(false, data->pwm_right); // Right motor
        }
    }
    
    ~BalanceMotorController() {
        if (initialized) {
            running = false;
            stopMotors();
            enableMotors(false);
            
            if (leftEncoderThread.joinable()) leftEncoderThread.join();
            if (rightEncoderThread.joinable()) rightEncoderThread.join();
            
            leftPwmPin.cleanup();
            rightPwmPin.cleanup();
        }
    }
};

int main() {
    // Set up signal handler for clean shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::cout << "Starting Wall-E Motor Control Process..." << std::endl;
    
    // Initialize shared memory (not as creator, IMU process should create it)
    SharedMemoryManager shm(false);
    g_shm = &shm;
    
    // Wait for IMU process to create shared memory
    int retry_count = 0;
    while (!shm.initialize() && retry_count < 50) {
        std::cout << "Waiting for IMU process to create shared memory..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        retry_count++;
    }
    
    if (retry_count >= 50) {
        std::cerr << "Failed to connect to shared memory. Is IMU process running?" << std::endl;
        return 1;
    }
    
    BalanceData* data = shm.getData();
    data->motor_active = true;
    
    // Initialize motor controller
    BalanceMotorController motors;
    if (!motors.initialize()) {
        std::cerr << "Failed to initialize motor controller" << std::endl;
        data->motor_active = false;
        return 1;
    }
    
    std::cout << "Motor control process running with 5ms cycle time..." << std::endl;
    
    // Wait for system initialization
    data->motion_mode = START;
    auto start_time = std::chrono::steady_clock::now();
    
    // Main control loop - 5ms cycle time (200Hz) for real-time response
    auto next_cycle = std::chrono::steady_clock::now();
    const auto cycle_duration = std::chrono::microseconds(CYCLE_TIME_MS * 1000);
    
    while (running) {
        auto cycle_start = std::chrono::steady_clock::now();
        
        // Check if IMU process is still active
        if (!data->imu_active) {
            std::cout << "IMU process inactive, stopping motors..." << std::endl;
            data->emergency_stop = true;
            motors.stopMotors();
            break;
        }
        
        // Handle startup sequence (2-second initialization period)
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(cycle_start - start_time).count();
        if (elapsed >= 2000 && data->motion_mode == START) { // 2 second startup delay
            if (data->kalman_angle >= BALANCE_ANGLE_MIN && data->kalman_angle <= BALANCE_ANGLE_MAX) {
                data->car_speed_integral = 0;
                data->setting_car_speed = 0;
                data->motion_mode = STANDBY;
                data->emergency_stop = false;
                std::cout << "System ready for balancing! Angle: " << data->kalman_angle << "°" << std::endl;
            } else {
                data->motion_mode = STOP;
                data->emergency_stop = true;
                std::cout << "Startup failed - robot not upright. Angle: " << data->kalman_angle << "°" << std::endl;
            }
        }
        
        // Run balance control
        motors.balanceControl(data);
        
        // Update timestamp
        data->motor_timestamp = getCurrentTimeMicros();
        
        // Debug output every 200ms (every 40 cycles) - include encoder data
        // static int debug_counter = 0;
        // if (++debug_counter >= 40) {
        //     debug_counter = 0;
        //     std::cout << "Mode: " << data->motion_mode << " | " 
        //               << "PWM L/R: " << (int)data->pwm_left << "/" << (int)data->pwm_right << " | "
        //               << "Speed: " << data->setting_car_speed << " | "
        //               << "Emergency: " << (data->emergency_stop ? "YES" : "NO") << std::endl;
        // }
        
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
    data->motor_active = false;
    data->emergency_stop = true;
    motors.stopMotors();
    motors.enableMotors(false);
    
    std::cout << "Motor control process shutdown complete." << std::endl;
    return 0;
}
