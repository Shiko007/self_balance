#include <iostream>
#include <thread>
#include <chrono>
#include <signal.h>
#include <iomanip>

#include "shared_memory.h"

// Global variables for signal handling
volatile bool running = true;
volatile bool shutting_down = false;
volatile bool force_exit = false;
SharedMemoryManager* g_shm = nullptr;

// Signal handler for clean shutdown
void signalHandler(int signal_num) {
    if (force_exit) {
        // Immediate exit if triple signal
        std::cout << "\nImmediate termination..." << std::endl;
        _exit(1);
    }
    
    if (shutting_down) {
        // Force exit if already shutting down and received second signal
        force_exit = true;
        std::cout << "\nForce exit..." << std::endl;
        _exit(1);
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
    }
    
    // Set a timeout for graceful shutdown
    std::thread timeout_thread([signal_num]() {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        if (shutting_down && signal_num == SIGINT) {
            std::cout << "\nTimeout reached - forcing exit..." << std::endl;
            _exit(1);
        }
    });
    timeout_thread.detach();
}

// Function to get a single character without blocking
char getChar() {
    char c = 0;
    // Simple blocking input for mode selection
    std::cin >> c;
    return c;
}

void printAutonomousInfo() {
    std::cout << "\n=== Wall-E Autonomous Mode ===" << std::endl;
    std::cout << "🤖 Robot operates independently" << std::endl;
    std::cout << "📹 Live streaming with arrow detection" << std::endl;
    std::cout << "🎯 Follows arrow directions automatically" << std::endl;
    std::cout << "🚧 Obstacle avoidance scenario available" << std::endl;
    std::cout << "⚠️  Only Ctrl+C to stop the system" << std::endl;
    std::cout << "\nAvailable modes:" << std::endl;
    std::cout << "  A - Arrow Detection Mode (default)" << std::endl;
    std::cout << "  S - Arrow-Guided Obstacle Avoidance Scenario" << std::endl;
    std::cout << "  Q - Quit (Ctrl+C)" << std::endl;
    std::cout << "\nEnter selection: ";
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
        case OBSTACLE_AVOIDANCE_SCENARIO: function_str = "OBSTACLE_SCENARIO"; break;
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

void runObstacleAvoidanceScenario(BalanceData* data) {
    std::cout << "\n🚧 Starting Arrow-Guided Obstacle Avoidance Scenario..." << std::endl;
    std::cout << "📏 Robot will move forward and detect obstacles at 50cm" << std::endl;
    std::cout << "🎯 When obstacle + arrow detected: Turn LEFT/RIGHT 90° (±5°), then continue" << std::endl;
    std::cout << "❌ When obstacle without arrow: Stop scenario" << std::endl;
    std::cout << "⚡ High-frequency monitoring (10ms) for instant response" << std::endl;
    std::cout << "📐 Using gyroscope integration for precise angle measurement" << std::endl;
    std::cout << "⚠️  Press Ctrl+C to stop the scenario" << std::endl;
    std::cout << std::endl;
    
    // Set scenario mode
    data->function_mode = OBSTACLE_AVOIDANCE_SCENARIO;
    data->ultrasonic_enabled = true;
    data->arrow_detection_enabled = true;  // Enable arrow detection for guidance
    data->emergency_stop = false;
    
    // Initialize motion parameters
    data->car_speed_integral = 0;
    data->setting_car_speed = 0;
    data->setting_turn_speed = 0;
    
    // Initialize persistent arrow direction (keeps last valid direction)
    ArrowDirection persistent_arrow_direction = ARROW_NONE;
    
    // Wait for ultrasonic sensor to be active
    std::cout << "⏳ Waiting for ultrasonic sensor..." << std::endl;
    int wait_count = 0;
    while (!data->ultrasonic_active && wait_count < 100) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        wait_count++;
    }
    
    if (!data->ultrasonic_active) {
        std::cout << "❌ Ultrasonic sensor not available - scenario aborted" << std::endl;
        data->motion_mode = STANDBY;
        return;
    }
    
    std::cout << "✓ Ultrasonic sensor ready!" << std::endl;
    std::cout << "🎯 Arrow detection enabled for guidance!" << std::endl;
    std::cout << "🚀 Starting forward motion..." << std::endl;
    std::cout << "📏 Monitoring distance - will react at 50cm..." << std::endl;
    
    // Start moving forward
    data->motion_mode = FORWARD;
    data->setting_car_speed = 60;  // Slightly slower for better reaction time
    
    bool scenario_active = true;
    bool obstacle_detected_at_50cm = false;
    auto scenario_start_time = std::chrono::steady_clock::now();
    
    // More frequent monitoring for immediate response
    while (scenario_active && running && !data->emergency_stop) {
        // Check current ultrasonic distance
        float current_distance = data->ultrasonic_distance_cm;
        
        // Continuously update persistent arrow direction when valid arrows are detected
        ArrowDirection current_arrow = data->detected_arrow_direction;
        if ((current_arrow == ARROW_LEFT || current_arrow == ARROW_RIGHT) && 
            data->arrow_confidence > 70.0f && 
            data->arrow_detection_active) {
            if (persistent_arrow_direction != current_arrow) {
                persistent_arrow_direction = current_arrow;
                std::cout << "🎯 Arrow direction updated: " << (persistent_arrow_direction == ARROW_LEFT ? "LEFT" : "RIGHT") << std::endl;
            }
        }
        
        // Display current distance every 20 cycles (200ms) for monitoring
        static int display_counter = 0;
        if (display_counter++ >= 20) {
            if (current_distance > 0) {
                std::string arrow_info = "";
                std::string persistent_info = "";
                
                // Current arrow info
                if (data->arrow_detection_active && data->detected_arrow_direction != ARROW_NONE) {
                    std::string arrow_dir = "";
                    switch (data->detected_arrow_direction) {
                        case ARROW_LEFT: arrow_dir = "LEFT"; break;
                        case ARROW_RIGHT: arrow_dir = "RIGHT"; break;
                        case ARROW_UP: arrow_dir = "UP"; break;
                        case ARROW_DOWN: arrow_dir = "DOWN"; break;
                        default: arrow_dir = "UNKNOWN"; break;
                    }
                    arrow_info = " | Current Arrow: " + arrow_dir + " (" + std::to_string((int)data->arrow_confidence) + "%)";
                }
                
                // Persistent arrow info
                if (persistent_arrow_direction != ARROW_NONE) {
                    std::string persist_dir = (persistent_arrow_direction == ARROW_LEFT) ? "LEFT" : "RIGHT";
                    persistent_info = " | Persistent: " + persist_dir;
                }
                
                std::cout << "📏 Distance: " << current_distance << " cm" << arrow_info << persistent_info << std::endl;
            }
            display_counter = 0;
        }
        
        // OBSTACLE DETECTED - Check for arrow guidance
        if (current_distance <= 50.0f && current_distance > 0) {
            std::cout << "🚧 OBSTACLE DETECTED AT 50cm THRESHOLD!" << std::endl;
            std::cout << "📏 Exact distance: " << current_distance << " cm" << std::endl;
            
            // Use persistent arrow direction for guidance (maintained from continuous monitoring)
            if (persistent_arrow_direction == ARROW_LEFT || persistent_arrow_direction == ARROW_RIGHT) {
                std::cout << "🧭 USING PERSISTENT ARROW GUIDANCE!" << std::endl;
                std::cout << "📍 Arrow direction: " << (persistent_arrow_direction == ARROW_LEFT ? "LEFT" : "RIGHT") << std::endl;
                std::cout << "🔄 Turning " << (persistent_arrow_direction == ARROW_LEFT ? "LEFT" : "RIGHT") << " 90 degrees..." << std::endl;
                
                // Stop forward motion and start turning
                data->motion_mode = (persistent_arrow_direction == ARROW_LEFT) ? TURNLEFT : TURNRIGHT;
                data->setting_car_speed = 0;  // Stop forward motion
                data->setting_turn_speed = 60; // Moderate turn speed for precise control
                
                // Record starting gyro_z position for reference
                float target_angle_change = 90.0f; // Target 90 degrees change
                float angle_tolerance = 5.0f; // ±5 degrees tolerance
                
                // Wait for gyro to stabilize and get initial reading
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                float initial_gyro_z = data->gyro_z;
                
                auto turn_start = std::chrono::steady_clock::now();
                
                std::cout << "📐 Starting turn measurement..." << std::endl;
                std::cout << "� Initial gyro_z: " << std::fixed << std::setprecision(1) << initial_gyro_z << "°" << std::endl;
                
                // Turn until we reach approximately 90 degrees difference from initial position
                while (true) {
                    float current_gyro_z = data->gyro_z;
                    float angle_difference = abs(current_gyro_z - initial_gyro_z);
                    
                    // Handle wraparound (if gyro goes from 359° to 1°, etc.)
                    if (angle_difference > 180.0f) {
                        angle_difference = 360.0f - angle_difference;
                    }
                    
                    // Check if we've reached the target
                    if (angle_difference >= (target_angle_change - angle_tolerance)) {
                        std::cout << "✅ Target angle reached: " << std::fixed << std::setprecision(1) << angle_difference << "°" << std::endl;
                        break;
                    }
                    
                    // Check for emergency conditions during turn
                    if (!running || data->emergency_stop || 
                        !data->imu_active || !data->motor_active) {
                        std::cout << "🚨 Emergency during turn - stopping!" << std::endl;
                        data->motion_mode = STANDBY;
                        data->setting_car_speed = 0;
                        data->setting_turn_speed = 0;
                        scenario_active = false;
                        break;
                    }
                    
                    // Safety timeout (maximum 5 seconds for a 90-degree turn)
                    auto current_time = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - turn_start).count();
                    if (elapsed > 5000) {
                        std::cout << "⏰ Turn timeout - completing turn" << std::endl;
                        break;
                    }
                    
                    // Show progress every 50ms
                    static int progress_counter = 0;
                    if (progress_counter++ >= 5) {
                        std::cout << "📐 Current angle difference: " << std::fixed << std::setprecision(1) << angle_difference << "° (target: " << target_angle_change << "°)" << std::endl;
                        progress_counter = 0;
                    }
                    
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                
                if (scenario_active) {
                    float final_gyro_z = data->gyro_z;
                    float final_angle_difference = abs(final_gyro_z - initial_gyro_z);
                    if (final_angle_difference > 180.0f) {
                        final_angle_difference = 360.0f - final_angle_difference;
                    }
                    
                    std::cout << "✅ Turn completed: " << std::fixed << std::setprecision(1) << final_angle_difference << "° - resuming forward motion" << std::endl;
                    
                    // Reset persistent arrow direction after successful turn
                    std::cout << "🔄 Resetting arrow guidance - ready for new direction" << std::endl;
                    persistent_arrow_direction = ARROW_NONE;
                    
                    // Resume forward motion
                    data->motion_mode = FORWARD;
                    data->setting_car_speed = 45;
                    data->setting_turn_speed = 0;
                    
                    // Reset display counter for distance monitoring
                    display_counter = 0;
                }
                
            } else {
                // No arrow guidance available (neither current nor persistent)
                std::cout << "❌ No arrow guidance available (current or persistent)" << std::endl;
                std::cout << "🛑 Stopping scenario - obstacle avoidance requires arrow guidance" << std::endl;
                
                data->motion_mode = STANDBY;
                data->setting_car_speed = 0;
                data->setting_turn_speed = 0;
                
                std::cout << "✅ Obstacle avoidance scenario completed!" << std::endl;
                obstacle_detected_at_50cm = true;
                scenario_active = false;
                break;
            }
        }
        
        // Safety timeout (30 seconds maximum)
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - scenario_start_time);
        if (elapsed_time.count() > 30) {
            std::cout << "⏰ Scenario timeout (30 seconds) - stopping for safety" << std::endl;
            data->motion_mode = STANDBY;
            data->setting_car_speed = 0;
            scenario_active = false;
        }
        
        // Check for system failures
        if (!data->imu_active || !data->motor_active || !data->ultrasonic_active) {
            std::cout << "🚨 System component failure - aborting scenario" << std::endl;
            if (!data->imu_active) std::cout << "❌ IMU system failure" << std::endl;
            if (!data->motor_active) std::cout << "❌ Motor system failure" << std::endl;
            if (!data->ultrasonic_active) std::cout << "❌ Ultrasonic system failure" << std::endl;
            data->motion_mode = STANDBY;
            data->setting_car_speed = 0;
            data->setting_turn_speed = 0;
            data->emergency_stop = true;
            scenario_active = false;
            break;
        }
        
        // Warn if arrow detection is not working (but don't abort - scenario can continue)
        if (!data->arrow_detection_active) {
            static int arrow_warning_counter = 0;
            if (arrow_warning_counter++ >= 100) {  // Warn every 1 second
                std::cout << "⚠️  Arrow detection not active - will stop at obstacles" << std::endl;
                arrow_warning_counter = 0;
            }
        }
        
        // Much faster monitoring for immediate response (10ms instead of 100ms)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Ensure robot is stopped
    data->motion_mode = STANDBY;
    data->setting_car_speed = 0;
    data->setting_turn_speed = 0;
    
    std::cout << "\n📊 Arrow-Guided Obstacle Avoidance Scenario completed:" << std::endl;
    std::cout << "   Final distance: " << data->ultrasonic_distance_cm << " cm" << std::endl;
    std::cout << "   Robot mode: STANDBY" << std::endl;
    std::cout << "   Obstacles encountered: " << (obstacle_detected_at_50cm ? "YES" : "NO") << std::endl;
    std::cout << "   Arrow detection was: " << (data->arrow_detection_active ? "ACTIVE" : "INACTIVE") << std::endl;
    std::cout << std::endl;
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
    
    // Show mode selection menu
    printAutonomousInfo();
    
    char mode_selection = getChar();
    std::cout << std::endl;
    
    switch (mode_selection) {
        case 'A':
        case 'a':
            std::cout << "🎯 Arrow Detection Mode selected" << std::endl;
            // Initialize autonomous mode (arrow detection)
            initializeAutonomousMode(data);
            break;
            
        case 'S':
        case 's':
            std::cout << "🚧 Arrow-Guided Obstacle Avoidance Scenario selected" << std::endl;
            // Initialize basic balancing first
            initializeAutonomousMode(data);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Let balancing stabilize
            // Run the scenario
            runObstacleAvoidanceScenario(data);
            // Return to arrow detection mode after scenario
            std::cout << "🔄 Returning to normal autonomous operation..." << std::endl;
            data->function_mode = ARROW_DETECTION;
            data->arrow_detection_enabled = true;
            break;
            
        case 'Q':
        case 'q':
            std::cout << "👋 Exiting..." << std::endl;
            return 0;
            
        default:
            std::cout << "⚠️  Invalid selection, defaulting to Arrow Detection Mode" << std::endl;
            initializeAutonomousMode(data);
            break;
    }
    
    // Variables to track status changes
    MotionMode last_motion_mode = data->motion_mode;
    FunctionMode last_function_mode = data->function_mode;
    ArrowDirection last_arrow_direction = data->detected_arrow_direction;
    bool last_emergency_stop = data->emergency_stop;
    bool last_imu_active = data->imu_active;
    bool last_motor_active = data->motor_active;
    bool last_arrow_detection_active = data->arrow_detection_active;
    bool last_ultrasonic_active = data->ultrasonic_active;
    bool last_obstacle_detected = data->obstacle_detected;
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
            data->arrow_detection_active != last_arrow_detection_active ||
            data->ultrasonic_active != last_ultrasonic_active ||
            data->obstacle_detected != last_obstacle_detected) {
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
            last_ultrasonic_active = data->ultrasonic_active;
            last_obstacle_detected = data->obstacle_detected;
        }
        
        // Check for system failures
        if (!data->imu_active || !data->motor_active) {
            std::cout << "\n🚨 SYSTEM FAILURE DETECTED!" << std::endl;
            if (!data->imu_active) std::cout << "❌ IMU process not responding!" << std::endl;
            if (!data->motor_active) std::cout << "❌ Motor process not responding!" << std::endl;
            std::cout << "🛑 Initiating emergency shutdown..." << std::endl;
            break;
        }
        
        // Special handling for obstacle avoidance scenario
        if (data->function_mode == OBSTACLE_AVOIDANCE_SCENARIO) {
            if (!data->ultrasonic_active) {
                std::cout << "\n🚨 ULTRASONIC SENSOR FAILURE during scenario!" << std::endl;
                std::cout << "🛑 Aborting scenario and stopping robot..." << std::endl;
                data->motion_mode = STANDBY;
                data->setting_car_speed = 0;
                data->emergency_stop = true;
                break;
            }
        }
        
        // Monitor for autonomous operation issues
        if (data->emergency_stop && !shutting_down) {
            std::cout << "\n🚨 EMERGENCY STOP ACTIVATED - Autonomous operation halted!" << std::endl;
            std::cout << "📐 Robot angle: " << data->kalman_angle << "°" << std::endl;
        }
        
        // Sleep in small chunks to be responsive to signals
        for (int i = 0; i < 5 && running && !shutting_down && !force_exit; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    // Clean shutdown sequence
    if (shutting_down) {
        std::cout << "\n🔧 Performing emergency stop and cleanup..." << std::endl;
    }
    
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
