#!/bin/bash

# Wall-E Autonomous Self-Balancing Robot Startup Script
# This script starts all necessary processes for autonomous operation
# Usage: ./wall_e.sh
#   Autonomous operation with arrow detection and live streaming

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[Wall-E Auto]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[Wall-E Auto]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[Wall-E Auto]${NC} $1"
}

print_error() {
    echo -e "${RED}[Wall-E Auto]${NC} $1"
}

print_stream() {
    echo -e "${CYAN}[Wall-E Auto]${NC} $1"
}

print_autonomous() {
    echo -e "${PURPLE}[Wall-E Auto]${NC} $1"
}

# Global flag to prevent duplicate cleanup
CLEANUP_CALLED=false

# Function to cleanup on exit
cleanup() {
    # Prevent duplicate cleanup calls
    if [ "$CLEANUP_CALLED" = true ]; then
        return 0
    fi
    CLEANUP_CALLED=true
    
    print_status "🛑 Shutdown signal received - stopping autonomous Wall-E system..."
    
    print_status "⏳ Gracefully stopping all processes..."
    
    # Stop all processes with SIGTERM first for graceful shutdown
    pkill -TERM -f imu_process 2>/dev/null && print_status "✓ IMU process stop signal sent" || true
    pkill -TERM -f motor_process 2>/dev/null && print_status "✓ Motor process stop signal sent" || true
    pkill -TERM -f control_interface 2>/dev/null && print_status "✓ Control interface stop signal sent" || true
    pkill -TERM -f arrow_detection_process 2>/dev/null && print_status "✓ Arrow detection stop signal sent" || true
    
    # Wait a moment for graceful shutdown
    sleep 2
    print_status "⏳ Waiting for processes to finish..."
    
    # Force kill any remaining processes
    pkill -KILL -f imu_process 2>/dev/null || true
    pkill -KILL -f motor_process 2>/dev/null || true
    pkill -KILL -f control_interface 2>/dev/null || true
    pkill -KILL -f arrow_detection_process 2>/dev/null || true
    
    # Clean up shared memory
    rm -f /dev/shm/wall_e_balance 2>/dev/null && print_status "✓ Shared memory cleaned" || true
    
    print_success "✅ Autonomous shutdown complete."
    exit 0
}

# Set trap for cleanup on script exit
trap cleanup SIGINT SIGTERM SIGQUIT SIGHUP EXIT

# Check if running as root (not recommended for safety)
if [ "$EUID" -eq 0 ]; then
    print_warning "Running as root is not recommended for safety reasons."
    print_warning "Consider running as a regular user in gpio, i2c groups."
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Change to script directory
cd "$(dirname "$0")"

print_autonomous "Starting Wall-E Autonomous Self-Balancing Robot"
print_autonomous "=============================================="
print_autonomous "🤖 AUTONOMOUS MODE - No manual controls"
print_autonomous "📹 Live arrow detection with streaming"
print_autonomous "🎯 Robot follows arrow directions automatically"
print_autonomous "⚠️  Only Ctrl+C to stop the system"
echo

# Check if executables exist
REQUIRED_EXES="imu_process motor_process control_interface arrow_detection_process"

for exe in $REQUIRED_EXES; do
    if [ ! -f "$exe" ]; then
        print_error "Executable $exe not found. Please run 'make all' first."
        exit 1
    fi
done

# Check I2C device
if [ ! -e "/dev/i2c-1" ]; then
    print_error "I2C device /dev/i2c-1 not found."
    print_error "Please enable I2C: sudo raspi-config -> Interface Options -> I2C"
    exit 1
fi

# Check for MPU6050 on I2C bus
print_status "Checking for MPU6050 sensor..."
if command -v i2cdetect >/dev/null 2>&1; then
    if i2cdetect -y 1 | grep -q "68"; then
        print_success "MPU6050 detected at address 0x68"
    else
        print_warning "MPU6050 not detected. Please check wiring."
        print_warning "Expected address: 0x68 on I2C bus 1"
        read -p "Continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
else
    print_warning "i2cdetect not available. Cannot verify MPU6050 connection."
fi

# Check PWM availability
print_status "Checking PWM availability..."
if [ -d "/sys/class/pwm/pwmchip0" ]; then
    print_success "PWM interface available"
else
    print_warning "PWM interface not found. Motor control may not work."
    print_warning "Add 'dtparam=pwm=on' to /boot/config.txt and reboot"
fi

# Check camera for arrow detection
print_status "Checking camera for arrow detection..."
if command -v rpicam-hello >/dev/null 2>&1; then
    if timeout 3s rpicam-hello --list-cameras >/dev/null 2>&1; then
        print_success "Camera detected and accessible"
    else
        print_error "Camera not detected or not accessible"
        print_error "Please enable camera: sudo raspi-config -> Interface Options -> Camera"
        print_error "Camera is required for autonomous operation!"
        exit 1
    fi
else
    print_error "rpicam tools not found. Camera is required for autonomous operation!"
    exit 1
fi

# Clean up any existing shared memory
rm -f /dev/shm/wall_e_balance 2>/dev/null || true

print_status "Starting autonomous system components..."
echo

# Start IMU process
print_status "Starting IMU process..."
./imu_process &
IMU_PID=$!

# Wait for IMU to initialize
sleep 2

# Check if IMU process is still running
if ! kill -0 $IMU_PID 2>/dev/null; then
    print_error "IMU process failed to start"
    exit 1
fi

print_success "IMU process started (PID: $IMU_PID)"

# Start motor process
print_status "Starting motor control process..."
./motor_process &
MOTOR_PID=$!

# Wait for motor process to initialize
sleep 2

# Check if motor process is still running
if ! kill -0 $MOTOR_PID 2>/dev/null; then
    print_error "Motor process failed to start"
    kill $IMU_PID 2>/dev/null || true
    exit 1
fi

print_success "Motor control process started (PID: $MOTOR_PID)"

# Start arrow detection process
print_status "Starting arrow detection with live streaming..."
./arrow_detection_process --stream &
ARROW_PID=$!

# Wait for arrow detection to initialize
sleep 3

# Check if arrow detection process is still running
if ! kill -0 $ARROW_PID 2>/dev/null; then
    print_error "Arrow detection process failed to start"
    print_error "Camera and arrow detection are required for autonomous operation!"
    kill $IMU_PID $MOTOR_PID 2>/dev/null || true
    exit 1
fi

print_success "Arrow detection process started (PID: $ARROW_PID)"

# Get IP address for streaming URL
PI_IP=$(hostname -I | awk '{print $1}')

# Display system information
echo
print_autonomous "🚀 AUTONOMOUS SYSTEM READY!"
echo
print_status "Process Information:"
echo "  IMU Process PID: $IMU_PID"
echo "  Motor Process PID: $MOTOR_PID"
echo "  Arrow Detection PID: $ARROW_PID"
echo
print_stream "📹 LIVE STREAMING ACTIVE!"
if [ ! -z "$PI_IP" ]; then
    print_stream "🌐 Stream URL: http://$PI_IP:8080"
    print_stream "📱 Open this URL in any web browser"
else
    print_stream "🌐 Stream available on port 8080"
    print_stream "📱 Use your Pi's IP address with port 8080"
fi
print_stream "🎯 Visual overlays show detected arrows and system status"
echo
print_autonomous "🤖 AUTONOMOUS OPERATION:"
print_autonomous "• Robot will automatically balance and follow arrow directions"
print_autonomous "• No manual control inputs available"
print_autonomous "• System monitors for failures and emergency conditions"
print_autonomous "• Live streaming shows real-time detection and status"
echo
print_warning "⚠️  SAFETY NOTICE:"
echo "- Keep the robot on a flat surface"
echo "- Ensure adequate space for movement"
echo "- Be ready to catch the robot if it falls"
echo "- Press Ctrl+C to stop the autonomous system"
if [ ! -z "$PI_IP" ]; then
    echo "- Live stream: http://$PI_IP:8080"
else
    echo "- Live stream available on port 8080"
fi
echo

# Start autonomous interface
print_status "Starting autonomous interface..."
./control_interface

# This line will be reached when control_interface exits
print_status "Autonomous interface stopped."
