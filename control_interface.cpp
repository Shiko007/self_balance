#include <iostream>
#include <thread>
#include <chrono>
#include <signal.h>
#include <iomanip>

#include "shared_memory.h"

// Global variables for signal handling
volatile bool running = true;
volatile bool shutting_down = false;
SharedMemoryManager* g_shm = nullptr;

// Signal handler for clean shutdown
void signalHandler(int signal_num) {
    if (shutting_down) {
        // Force exit if already shutting down
        std::cout << "\nForce exit..." << std::endl;
        exit(1);
    }
    
    shutting_down = true;
    running = false;
    
    // Only print detailed shutdown message for user-initiated signals (SIGINT)
    // For SIGTERM (sent by wall_e.sh), just shut down quietly to avoid duplicates
    if (signal_num == SIGINT) {
        std::cout << "\n\n🛑 Shutdown signal received (Ctrl+C)" << std::endl;
        std::cout << "🔄 Initiating clean autonomous system shutdown..." << std::endl;
        std::cout << "⚠️  Please wait for all processes to stop safely" << std::endl;
        std::cout << "📡 Signal: SIGINT (Ctrl+C)" << std::endl;
        std::cout << std::endl;
    } else {
        // For other signals (like SIGTERM from wall_e.sh), just acknowledge quietly
        // The main wall_e.sh script will handle the user messaging
    }
}

// Function to get a single character without blocking
char getChar() {
    // Always return 0 - no user input in autonomous mode
    return 0;
}

void printAutonomousInfo() {
    std::cout << "\n=== Wall-E Autonomous Mode ===" << std::endl;
    std::cout << "🤖 Robot operates independently" << std::endl;
    std::cout << "📹 Live streaming with arrow detection" << std::endl;
    std::cout << "🎯 Follows arrow directions automatically" << std::endl;
    std::cout << "⚠️  Only Ctrl+C to stop the system" << std::endl;
    std::cout << "\nPress Ctrl+C to stop autonomous operation:" << std::endl;
}

std::string arrowDirectionToString(ArrowDirection dir) {
    switch (dir) {
        case ARROW_LEFT: return "LEFT";
        case ARROW_RIGHT: return "RIGHT";
        case ARROW_UP: return "UP";
        case ARROW_DOWN: return "DOWN";
        case ARROW_UNKNOWN: return "UNKNOWN";
        default: return "NONE";
    }
}

void printStatus(BalanceData* data) {
    std::cout << "Status: "; // Remove carriage return for event-based logging
    std::cout << "Angle: " << std::fixed << std::setprecision(1) << data->kalman_angle << "° | ";
    
    std::string mode_str;
    switch (data->motion_mode) {
        case STANDBY: mode_str = "STANDBY"; break;
        case FORWARD: mode_str = "FORWARD"; break;
        case BACKWARD: mode_str = "BACKWARD"; break;
        case TURNLEFT: mode_str = "TURN_LEFT"; break;
        case TURNRIGHT: mode_str = "TURN_RIGHT"; break;
        case STOP: mode_str = "STOP"; break;
        case START: mode_str = "START"; break;
        default: mode_str = "UNKNOWN"; break;
    }
    
    std::string function_str;
    switch (data->function_mode) {
        case IDLE: function_str = "IDLE"; break;
        case IRREMOTE: function_str = "IRREMOTE"; break;
        case OBSTACLE: function_str = "OBSTACLE"; break;
        case FOLLOW: function_str = "FOLLOW"; break;
        case BLUETOOTH: function_str = "BLUETOOTH"; break;
        case FOLLOW2: function_str = "FOLLOW2"; break;
        case ARROW_DETECTION: function_str = "ARROW_DETECTION"; break;
        default: function_str = "UNKNOWN"; break;
    }
    
    std::cout << "Mode: " << mode_str << " | ";
    std::cout << "Function: " << function_str << " | ";
    std::cout << "PWM L/R: " << (int)data->pwm_left << "/" << (int)data->pwm_right << " | ";
    std::cout << "Speed: " << data->setting_car_speed << " | ";
    std::cout << "Turn: " << data->setting_turn_speed << " | ";
    std::cout << "IMU: " << (data->imu_active ? "OK" : "FAIL") << " | ";
    std::cout << "MOTOR: " << (data->motor_active ? "OK" : "FAIL") << " | ";
    std::cout << "ARROW: " << (data->arrow_detection_active ? "OK" : "OFF") << " | ";
    if (data->arrow_detection_active) {
        std::cout << "DIR: " << arrowDirectionToString(data->detected_arrow_direction) 
                  << "(" << std::setprecision(0) << data->arrow_confidence << "%) | ";
    }
    std::cout << "ULTRASONIC: " << (data->ultrasonic_active ? "OK" : "OFF") << " | ";
    if (data->ultrasonic_active) {
        std::cout << "DIST: " << std::setprecision(1) << data->ultrasonic_distance_cm << "cm | ";
        std::cout << "OBSTACLE: " << (data->obstacle_detected ? "YES" : "NO") << " | ";
    }
    std::cout << "EMERGENCY: " << (data->emergency_stop ? "YES" : "NO") << " | ";
    std::cout << "SAFETY: " << (data->safety_override ? "ON" : "OFF");
    // Remove flush() and trailing spaces since we're doing event-based logging
}

void initializeAutonomousMode(BalanceData* data) {
    // Set up the robot for autonomous operation
    std::cout << "\n=== Initializing Autonomous Mode ===" << std::endl;
    
    // Enable arrow detection for autonomous navigation
    data->arrow_detection_enabled = true;
    data->function_mode = ARROW_DETECTION;
    
    // Enable ultrasonic sensor for obstacle detection
    data->ultrasonic_enabled = true;
    
    // Clear any emergency states
    data->emergency_stop = false;
    data->safety_override = false;
    
    // Initialize motion parameters EXACTLY like the original manual sequence
    data->car_speed_integral = 0;  // Reset integral
    data->setting_car_speed = 0;   // No initial speed
    data->setting_turn_speed = 0;  // No initial turn
    
    // WAIT for motor process initialization before starting balancing
    std::cout << "⏳ Waiting for motor process initialization..." << std::endl;
    int wait_count = 0;
    while (!data->motor_active && wait_count < 100) {  // Wait up to 10 seconds
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        wait_count++;
    }
    
    if (!data->motor_active) {
        std::cout << "❌ Motor process not ready - starting in emergency mode" << std::endl;
        data->motion_mode = STOP;
        data->emergency_stop = true;
        return;
    }
    
    std::cout << "✓ Motor process ready!" << std::endl;
    
    // CRITICAL: Start balancing sequence exactly like manual '5' key press
    // But wait a bit more to ensure all processes are fully ready
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    data->motion_mode = START;
    data->emergency_stop = false;  // Ensure emergency is cleared
    
    std::cout << "✓ Arrow detection enabled" << std::endl;
    std::cout << "✓ Function mode set to ARROW_DETECTION" << std::endl;
    std::cout << "✓ Emergency states cleared" << std::endl;
    std::cout << "✓ Balancing sequence initiated (START mode)" << std::endl;
    std::cout << "✓ Motion parameters initialized" << std::endl;
    std::cout << "\n🤖 Wall-E is now operating autonomously!" << std::endl;
    std::cout << "📹 Live streaming active (if arrow detection process is running)" << std::endl;
    std::cout << "🎯 Robot will follow arrow directions automatically" << std::endl;
    std::cout << "\nPress Ctrl+C to stop the autonomous system.\n" << std::endl;
}

int main() {
    // Set up signal handlers for clean shutdown
    signal(SIGINT, signalHandler);   // Ctrl+C
    signal(SIGTERM, signalHandler);  // Termination request
    signal(SIGQUIT, signalHandler);  // Ctrl+'\'
    signal(SIGHUP, signalHandler);   // Terminal hangup
    
    std::cout << "Starting Wall-E Autonomous Interface..." << std::endl;
    std::cout << "🤖 Autonomous operation mode active" << std::endl;
    std::cout << "⚠️  Press Ctrl+C for clean shutdown" << std::endl;
    
    // Initialize shared memory (not as creator)
    SharedMemoryManager shm(false);
    g_shm = &shm;
    
    // Wait for shared memory to be available
    int retry_count = 0;
    while (!shm.initialize() && retry_count < 50) {
        std::cout << "Waiting for system processes..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        retry_count++;
    }
    
    if (retry_count >= 50) {
        std::cerr << "Failed to connect to shared memory. Are IMU and motor processes running?" << std::endl;
        return 1;
    }
    
    BalanceData* data = shm.getData();
    
    // Initialize autonomous mode
    initializeAutonomousMode(data);
    
    // Variables to track status changes
    MotionMode last_motion_mode = data->motion_mode;
    FunctionMode last_function_mode = data->function_mode;
    ArrowDirection last_arrow_direction = data->detected_arrow_direction;
    bool last_emergency_stop = data->emergency_stop;
    bool last_imu_active = data->imu_active;
    bool last_motor_active = data->motor_active;
    bool last_arrow_detection_active = data->arrow_detection_active;
    bool status_printed = false;
    
    while (running) {
        // No manual input processing - autonomous operation only
        
        // Check for significant status changes
        bool status_changed = false;
        if (data->motion_mode != last_motion_mode ||
            data->function_mode != last_function_mode ||
            data->detected_arrow_direction != last_arrow_direction ||
            data->emergency_stop != last_emergency_stop ||
            data->imu_active != last_imu_active ||
            data->motor_active != last_motor_active ||
            data->arrow_detection_active != last_arrow_detection_active) {
            status_changed = true;
        }
        
        // Print status only on changes or first time
        if ((status_changed || !status_printed) && !shutting_down) {
            if (!status_printed) {
                std::cout << "\nSystem Status Updates (only on changes):" << std::endl;
                status_printed = true;
            }
            //printStatus(data);
            std::cout << std::endl; // Add newline after status change
            
            // Update last known values
            last_motion_mode = data->motion_mode;
            last_function_mode = data->function_mode;
            last_arrow_direction = data->detected_arrow_direction;
            last_emergency_stop = data->emergency_stop;
            last_imu_active = data->imu_active;
            last_motor_active = data->motor_active;
            last_arrow_detection_active = data->arrow_detection_active;
        }
        
        // Check for system failures
        if (!data->imu_active || !data->motor_active) {
            std::cout << "\n🚨 SYSTEM FAILURE DETECTED!" << std::endl;
            if (!data->imu_active) std::cout << "❌ IMU process not responding!" << std::endl;
            if (!data->motor_active) std::cout << "❌ Motor process not responding!" << std::endl;
            std::cout << "🛑 Initiating emergency shutdown..." << std::endl;
            break;
        }
        
        // Monitor for autonomous operation issues
        if (data->emergency_stop && !shutting_down) {
            std::cout << "\n🚨 EMERGENCY STOP ACTIVATED - Autonomous operation halted!" << std::endl;
            std::cout << "📐 Robot angle: " << data->kalman_angle << "°" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    // Clean shutdown sequence
    std::cout << "\n🔧 Performing emergency stop and cleanup..." << std::endl;
    
    // Emergency stop before exit
    if (data) {
        data->emergency_stop = true;
        data->motion_mode = STOP;
        data->setting_car_speed = 0;
        data->setting_turn_speed = 0;
        data->arrow_detection_enabled = false;
        std::cout << "✓ Robot motion stopped" << std::endl;
        std::cout << "✓ Arrow detection disabled" << std::endl;
    }
    
    // Give other processes time to notice the emergency stop
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "\n✅ Autonomous interface shutdown complete." << std::endl;
    std::cout << "📝 Note: Other processes (IMU, Motor, Arrow Detection) should stop automatically." << std::endl;
    return 0;
}
