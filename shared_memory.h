#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <cstdio>
#include <new>
#include <time.h>

// Shared memory name
#define SHM_NAME "/wall_e_balance"

// Balance control parameters (optimized values)
#define BALANCE_ANGLE_MIN -22.0f
#define BALANCE_ANGLE_MAX 22.0f

// PID parameters (tuned for optimal performance)
#define KP_BALANCE 55.0f
#define KD_BALANCE 0.75f
#define KP_SPEED 10.0f
#define KI_SPEED 0.26f
#define KP_TURN 2.5f
#define KD_TURN 0.5f

// Timing parameters (5ms cycle for real-time control)
#define CYCLE_TIME_MS 5
#define SPEED_CONTROL_PERIOD 8  // Speed control every 8 cycles (40ms)

// Motion modes
enum MotionMode {
    STANDBY = 0,
    FORWARD = 1,
    BACKWARD = 2,
    TURNLEFT = 3,
    TURNRIGHT = 4,
    STOP = 5,
    START = 6
};

// Function modes
enum FunctionMode {
    IDLE = 0,
    IRREMOTE = 1,
    OBSTACLE = 2,
    FOLLOW = 3,
    BLUETOOTH = 4,
    FOLLOW2 = 5,
    ARROW_DETECTION = 6,
    OBSTACLE_AVOIDANCE_SCENARIO = 7
};

// Arrow direction enumeration
enum ArrowDirection {
    ARROW_NONE = 0,
    ARROW_LEFT = 1,
    ARROW_RIGHT = 2,
    ARROW_UP = 3,
    ARROW_DOWN = 4,
    ARROW_UNKNOWN = 5
};

// Shared memory structure
struct BalanceData {
    // IMU data
    float kalman_angle;           // Filtered angle from Kalman filter
    float gyro_x;                 // X-axis gyro (for balance control)
    float gyro_z;                 // Z-axis gyro (for rotation control)
    
    // Control parameters
    float angle_zero;             // Angle calibration offset
    float angular_velocity_zero;  // Angular velocity calibration offset
    
    // Motor control
    float pwm_left;               // Left motor PWM (-255 to 255)
    float pwm_right;              // Right motor PWM (-255 to 255)
    
    // Speed and rotation settings
    int setting_car_speed;        // Desired car speed
    int setting_turn_speed;       // Desired turn speed
    
    // System state
    volatile MotionMode motion_mode;     // Current motion mode
    volatile FunctionMode function_mode; // Current function mode
    volatile bool emergency_stop;        // Emergency stop flag
    volatile bool system_initialized;    // System initialization flag
    
    // Encoder data
    volatile long encoder_left_count;    // Left encoder count
    volatile long encoder_right_count;   // Right encoder count
    
    // Speed control variables
    double car_speed_integral;           // Speed integral for PID
    double speed_filter;                 // Filtered speed
    int speed_control_period_count;      // Counter for speed control period
    
    // Timestamps for synchronization
    volatile uint64_t imu_timestamp;     // Last IMU update timestamp (microseconds)
    volatile uint64_t motor_timestamp;   // Last motor update timestamp (microseconds)
    
    // Debug/status
    bool imu_active;                     // IMU process is running
    bool motor_active;                   // Motor process is running
    
    // Safety
    volatile bool safety_override;       // Manual safety override
    
    // Arrow detection data
    volatile ArrowDirection detected_arrow_direction;  // Last detected arrow direction
    volatile float arrow_confidence;                   // Confidence of arrow detection (0-100)
    volatile uint64_t arrow_timestamp;                 // Last arrow detection timestamp (microseconds)
    volatile bool arrow_detection_active;              // Arrow detection process is running
    volatile bool arrow_detection_enabled;             // Arrow detection functionality enabled
    
    // Ultrasonic sensor data
    volatile float ultrasonic_distance_cm;             // Distance measurement in centimeters
    volatile uint64_t ultrasonic_timestamp;            // Last ultrasonic reading timestamp (microseconds)
    volatile bool ultrasonic_active;                   // Ultrasonic process is running
    volatile bool ultrasonic_enabled;                  // Ultrasonic functionality enabled
    volatile float ultrasonic_min_distance;            // Minimum safe distance (cm)
    volatile bool obstacle_detected;                   // Obstacle within minimum distance
    
    // Constructor to initialize values
    BalanceData() {
        memset(this, 0, sizeof(BalanceData));
        kalman_angle = 0.0f;
        gyro_x = 0.0f;
        gyro_z = 0.0f;
        angle_zero = 0.0f;
        angular_velocity_zero = 0.0f;
        pwm_left = 0.0f;
        pwm_right = 0.0f;
        setting_car_speed = 0;
        setting_turn_speed = 0;
        motion_mode = START;
        function_mode = IDLE;
        emergency_stop = false;
        system_initialized = false;
        encoder_left_count = 0;
        encoder_right_count = 0;
        car_speed_integral = 0.0;
        speed_filter = 0.0;
        speed_control_period_count = 0;
        imu_timestamp = 0;
        motor_timestamp = 0;
        imu_active = false;
        motor_active = false;
        safety_override = false;
        detected_arrow_direction = ARROW_NONE;
        arrow_confidence = 0.0f;
        arrow_timestamp = 0;
        arrow_detection_active = false;
        arrow_detection_enabled = false;
        ultrasonic_distance_cm = 999.0f;  // Initialize to max value
        ultrasonic_timestamp = 0;
        ultrasonic_active = false;
        ultrasonic_enabled = true;  // Enable by default
        ultrasonic_min_distance = 20.0f;  // 20cm minimum safe distance
        obstacle_detected = false;
    }
};

// Shared memory management class
class SharedMemoryManager {
private:
    int shm_fd;
    BalanceData* data;
    bool is_creator;
    
public:
    SharedMemoryManager(bool create = false) : shm_fd(-1), data(nullptr), is_creator(create) {}
    
    bool initialize() {
        if (is_creator) {
            // Create shared memory
            shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
            if (shm_fd == -1) {
                perror("shm_open create");
                return false;
            }
            
            // Set size
            if (ftruncate(shm_fd, sizeof(BalanceData)) == -1) {
                perror("ftruncate");
                close(shm_fd);
                return false;
            }
        } else {
            // Open existing shared memory
            shm_fd = shm_open(SHM_NAME, O_RDWR, 0666);
            if (shm_fd == -1) {
                perror("shm_open open");
                return false;
            }
        }
        
        // Map memory
        data = (BalanceData*)mmap(nullptr, sizeof(BalanceData), 
                                  PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (data == MAP_FAILED) {
            perror("mmap");
            close(shm_fd);
            return false;
        }
        
        // Initialize data if creator
        if (is_creator) {
            new(data) BalanceData();  // Placement new to call constructor
        }
        
        return true;
    }
    
    BalanceData* getData() { return data; }
    
    void cleanup() {
        if (data != nullptr) {
            munmap(data, sizeof(BalanceData));
            data = nullptr;
        }
        
        if (shm_fd != -1) {
            close(shm_fd);
            shm_fd = -1;
        }
        
        if (is_creator) {
            shm_unlink(SHM_NAME);
        }
    }
    
    ~SharedMemoryManager() {
        cleanup();
    }
};

// Utility functions
inline uint64_t getCurrentTimeMicros() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

inline void constrainValue(float& value, float min_val, float max_val) {
    if (value < min_val) value = min_val;
    if (value > max_val) value = max_val;
}

inline void constrainValue(double& value, double min_val, double max_val) {
    if (value < min_val) value = min_val;
    if (value > max_val) value = max_val;
}

#endif // SHARED_MEMORY_H
