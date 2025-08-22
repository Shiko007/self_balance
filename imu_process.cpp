#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <cstdint>
#include <cmath>
#include <chrono>
#include <thread>
#include <signal.h>
#include <csignal>

extern "C" {
#include <i2c/smbus.h>
}

#include "shared_memory.h"

// MPU6050 registers
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48

// Global variables for signal handling
volatile bool running = true;
SharedMemoryManager* g_shm = nullptr;

// Signal handler for clean shutdown
void signalHandler(int signal __attribute__((unused))) {
    std::cout << "\nShutting down IMU process..." << std::endl;
    running = false;
    if (g_shm && g_shm->getData()) {
        g_shm->getData()->imu_active = false;
    }
}

// Kalman Filter class (optimized implementation)
class KalmanFilter {
private:
    float angle;
    float q_bias;
    float angle_err;
    float Pdot[4];
    float P[2][2];
    float PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
    float angle_dot;
    
public:
    float Gyro_x, Gyro_y, Gyro_z;
    
    KalmanFilter() {
        angle = 0.0f;
        q_bias = 0.0f;
        angle_err = 0.0f;
        Gyro_x = 0.0f;
        Gyro_y = 0.0f;
        Gyro_z = 0.0f;
        angle_dot = 0.0f;
        
        Pdot[0] = Pdot[1] = Pdot[2] = Pdot[3] = 0.0f;
        P[0][0] = P[1][1] = 1.0f;
        P[0][1] = P[1][0] = 0.0f;
        
        PCt_0 = PCt_1 = E = K_0 = K_1 = t_0 = t_1 = 0.0f;
    }
    
    void Kalman_Filter(double angle_m, double gyro_m, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0) {
        angle += (gyro_m - q_bias) * dt;
        angle_err = angle_m - angle;
        Pdot[0] = Q_angle - P[0][1] - P[1][0];
        Pdot[1] = -P[1][1];
        Pdot[2] = -P[1][1];
        Pdot[3] = Q_gyro;
        P[0][0] += Pdot[0] * dt;
        P[0][1] += Pdot[1] * dt;
        P[1][0] += Pdot[2] * dt;
        P[1][1] += Pdot[3] * dt;
        PCt_0 = C_0 * P[0][0];
        PCt_1 = C_0 * P[1][0];
        E = R_angle + C_0 * PCt_0;
        K_0 = PCt_0 / E;
        K_1 = PCt_1 / E;
        t_0 = PCt_0;
        t_1 = C_0 * P[0][1];
        P[0][0] -= K_0 * t_0;
        P[0][1] -= K_0 * t_1;
        P[1][0] -= K_1 * t_0;
        P[1][1] -= K_1 * t_1;
        angle += K_0 * angle_err;
        q_bias += K_1 * angle_err;
        angle_dot = gyro_m - q_bias;
    }
    
    void Angle(int16_t ax __attribute__((unused)), int16_t ay, int16_t az, int16_t gx, int16_t gy __attribute__((unused)), int16_t gz,
               float dt, float Q_angle, float Q_gyro, float R_angle, float C_0, float K1 __attribute__((unused))) {
        float Angle = atan2(ay, az) * 57.3f;  // Convert to degrees
        Gyro_x = (gx - 128.1f) / 131.0f;     // Convert to degrees/second (calibrated offset)
        Kalman_Filter(Angle, Gyro_x, dt, Q_angle, Q_gyro, R_angle, C_0);
        Gyro_z = -gz / 131.0f;               // Convert to degrees/second
    }
    
    float getAngle() const { return angle; }
    void setAngle(float new_angle) { angle = new_angle; }
};

int16_t read_word_2c(int fd, uint8_t reg) {
    uint8_t high = i2c_smbus_read_byte_data(fd, reg);
    uint8_t low = i2c_smbus_read_byte_data(fd, reg + 1);
    int16_t value = (high << 8) + low;
    return value;
}

bool initializeMPU6050(int fd) {
    // Wake up the MPU6050 (it starts in sleep mode)
    if (i2c_smbus_write_byte_data(fd, PWR_MGMT_1, 0) < 0) {
        std::cerr << "Failed to wake up MPU6050" << std::endl;
        return false;
    }

    // Set gyro range to ±250°/s (default)
    if (i2c_smbus_write_byte_data(fd, GYRO_CONFIG, 0) < 0) {
        std::cerr << "Failed to configure gyroscope" << std::endl;
        return false;
    }

    // Set accelerometer range to ±2g (default)
    if (i2c_smbus_write_byte_data(fd, ACCEL_CONFIG, 0) < 0) {
        std::cerr << "Failed to configure accelerometer" << std::endl;
        return false;
    }

    std::cout << "MPU6050 initialized successfully!" << std::endl;
    return true;
}

int main() {
    // Set up signal handler for clean shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::cout << "Starting Wall-E IMU Process..." << std::endl;
    
    // Initialize shared memory (as creator)
    SharedMemoryManager shm(true);
    g_shm = &shm;
    
    if (!shm.initialize()) {
        std::cerr << "Failed to initialize shared memory" << std::endl;
        return 1;
    }
    
    BalanceData* data = shm.getData();
    data->imu_active = true;
    
    // Open I2C bus
    int fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open I2C bus" << std::endl;
        data->imu_active = false;
        return 1;
    }

    // Set I2C slave address to MPU6050
    if (ioctl(fd, I2C_SLAVE, MPU6050_ADDR) < 0) {
        std::cerr << "Failed to set I2C slave address" << std::endl;
        close(fd);
        data->imu_active = false;
        return 1;
    }

    // Initialize MPU6050
    if (!initializeMPU6050(fd)) {
        close(fd);
        data->imu_active = false;
        return 1;
    }

    // Initialize Kalman filter
    KalmanFilter kalmanfilter;
    
    // Kalman filter parameters (optimized for stability)
    const float dt = 0.005f;  // 5ms cycle time
    const float Q_angle = 0.001f;
    const float Q_gyro = 0.005f;
    const float R_angle = 0.5f;
    const float C_0 = 1.0f;
    const float K1 = 0.05f;
    
    std::cout << "IMU process running with 5ms cycle time..." << std::endl;
    
    // Main loop - 5ms cycle time (200Hz) for real-time control
    auto next_cycle = std::chrono::steady_clock::now();
    const auto cycle_duration = std::chrono::microseconds(CYCLE_TIME_MS * 1000);
    
    while (running) {
        // Read accelerometer data
        int16_t ax = read_word_2c(fd, ACCEL_XOUT_H);
        int16_t ay = read_word_2c(fd, ACCEL_YOUT_H);
        int16_t az = read_word_2c(fd, ACCEL_ZOUT_H);
        
        // Read gyroscope data
        int16_t gx = read_word_2c(fd, GYRO_XOUT_H);
        int16_t gy = read_word_2c(fd, GYRO_YOUT_H);
        int16_t gz = read_word_2c(fd, GYRO_ZOUT_H);

        // Apply Kalman filter (sensor fusion)
        kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
        
        // Update shared memory
        data->kalman_angle = kalmanfilter.getAngle();
        data->gyro_x = kalmanfilter.Gyro_x;
        data->gyro_z = kalmanfilter.Gyro_z;
        data->imu_timestamp = getCurrentTimeMicros();
        
        // Check for emergency stop conditions (safety limits)
        if (data->motion_mode != START && data->motion_mode != STOP) {
            if (data->kalman_angle < BALANCE_ANGLE_MIN || data->kalman_angle > BALANCE_ANGLE_MAX) {
                data->emergency_stop = true;
                data->motion_mode = STOP;
                std::cout << "Emergency stop triggered! Angle: " << data->kalman_angle << "°" << std::endl;
            }
        }
        
        // Debug output every 100ms (every 20 cycles)
        // static int debug_counter = 0;
        // if (++debug_counter >= 20) {
        //     debug_counter = 0;
        //     std::cout << "Angle: " << data->kalman_angle << "° | " 
        //               << "Gyro_X: " << data->gyro_x << "°/s | "
        //               << "Gyro_Z: " << data->gyro_z << "°/s | "
        //               << "Emergency: " << (data->emergency_stop ? "YES" : "NO") << std::endl;
        // }
        
        // Precise timing control
        next_cycle += cycle_duration;
        std::this_thread::sleep_until(next_cycle);
        
        // Check if we're falling behind
        auto now = std::chrono::steady_clock::now();
        if (now > next_cycle) {
            // Reset timing if we're too far behind
            next_cycle = now;
        }
    }
    
    // Cleanup
    data->imu_active = false;
    data->emergency_stop = true;
    close(fd);
    
    std::cout << "IMU process shutdown complete." << std::endl;
    return 0;
}
