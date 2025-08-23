# Wall-E Self-Balancing Robot Makefile
# Raspberry Pi 5 compatible

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -pthread
LIBS = -li2c -lrt
OPENCV_FLAGS = `pkg-config --cflags --libs opencv4`

# Target binaries
TARGETS = imu_process motor_process control_interface arrow_detection_process ultrasonic_process

# Source files
IMU_SRC = imu_process.cpp
MOTOR_SRC = motor_process.cpp
CONTROL_SRC = control_interface.cpp
ARROW_SRC = arrow_detection_process.cpp
ULTRASONIC_SRC = ultrasonic_process.cpp

# Header files
HEADERS = shared_memory.h

# Build all targets
all: $(TARGETS)

# Build individual targets
imu_process: $(IMU_SRC) $(HEADERS)
	@echo "Building IMU process..."
	$(CXX) $(CXXFLAGS) $(IMU_SRC) -o imu_process $(LIBS)

motor_process: $(MOTOR_SRC) $(HEADERS)
	@echo "Building motor control process..."
	$(CXX) $(CXXFLAGS) $(MOTOR_SRC) -o motor_process $(LIBS)

control_interface: $(CONTROL_SRC) $(HEADERS)
	@echo "Building control interface..."
	$(CXX) $(CXXFLAGS) $(CONTROL_SRC) -o control_interface $(LIBS)

arrow_detection_process: $(ARROW_SRC) $(HEADERS)
	@echo "Building arrow detection process..."
	$(CXX) $(CXXFLAGS) $(ARROW_SRC) -o arrow_detection_process $(LIBS) $(OPENCV_FLAGS)

ultrasonic_process: $(ULTRASONIC_SRC) $(HEADERS)
	@echo "Building ultrasonic sensor process..."
	$(CXX) $(CXXFLAGS) $(ULTRASONIC_SRC) -o ultrasonic_process $(LIBS)

# Install and setup
install: all
	@echo "Installing Wall-E system..."
	sudo cp imu_process /usr/local/bin/
	sudo cp motor_process /usr/local/bin/
	sudo cp control_interface /usr/local/bin/
	sudo cp arrow_detection_process /usr/local/bin/
	sudo cp ultrasonic_process /usr/local/bin/
	@echo "Installation complete!"

# Setup system permissions
setup-permissions:
	@echo "Setting up system permissions for GPIO and I2C..."
	sudo usermod -a -G gpio,i2c,dialout $(USER)
	@echo "You may need to log out and back in for group changes to take effect"

# Enable I2C if not already enabled
setup-i2c:
	@echo "Checking I2C configuration..."
	@if ! grep -q "^dtparam=i2c_arm=on" /boot/config.txt; then \
		echo "Enabling I2C..."; \
		sudo bash -c 'echo "dtparam=i2c_arm=on" >> /boot/config.txt'; \
		echo "I2C enabled. Please reboot to apply changes."; \
	else \
		echo "I2C is already enabled."; \
	fi

# Enable PWM for motor control
setup-pwm:
	@echo "Checking PWM configuration..."
	@if ! grep -q "^dtparam=pwm=on" /boot/config.txt; then \
		echo "Enabling PWM..."; \
		sudo bash -c 'echo "dtparam=pwm=on" >> /boot/config.txt'; \
		echo "PWM enabled. Please reboot to apply changes."; \
	else \
		echo "PWM is already enabled."; \
	fi

# Complete system setup
setup: setup-permissions setup-i2c setup-pwm
	@echo ""
	@echo "System setup complete!"
	@echo "If this is the first time running setup, please reboot your Raspberry Pi."
	@echo ""
	@echo "After reboot, you can:"
	@echo "1. Start the system: make run"
	@echo "2. Or start processes individually:"
	@echo "   - IMU process: ./imu_process"
	@echo "   - Motor process: ./motor_process"
	@echo "   - Control interface: ./control_interface"

# Check system requirements
check-deps:
	@echo "Checking system dependencies..."
	@echo -n "Checking for i2c-tools... "
	@if command -v i2cdetect >/dev/null 2>&1; then \
		echo "OK"; \
	else \
		echo "MISSING"; \
		echo "Install with: sudo apt install i2c-tools"; \
	fi
	@echo -n "Checking for libi2c-dev... "
	@if ldconfig -p | grep -q libi2c; then \
		echo "OK"; \
	else \
		echo "MISSING"; \
		echo "Install with: sudo apt install libi2c-dev"; \
	fi
	@echo -n "Checking for OpenCV... "
	@if pkg-config --exists opencv4; then \
		echo "OK"; \
	else \
		echo "MISSING"; \
		echo "Install with: sudo apt install libopencv-dev"; \
	fi
	@echo -n "Checking I2C device... "
	@if [ -e /dev/i2c-1 ]; then \
		echo "OK"; \
	else \
		echo "MISSING"; \
		echo "I2C not enabled. Run: make setup-i2c"; \
	fi

# Test I2C connection to MPU6050
test-i2c:
	@echo "Testing I2C connection to MPU6050..."
	@if command -v i2cdetect >/dev/null 2>&1; then \
		echo "Scanning I2C bus 1 for MPU6050 (address 0x68):"; \
		i2cdetect -y 1; \
	else \
		echo "i2cdetect not found. Install i2c-tools: sudo apt install i2c-tools"; \
	fi

# Run the complete system (now autonomous by default)
run: all
	@echo "Starting Wall-E autonomous self-balancing robot system..."
	@./wall_e.sh

# Run the complete system with arrow detection (legacy compatibility)
run-with-arrows: all
	@echo "Starting Wall-E autonomous self-balancing robot system..."
	@./wall_e.sh

# Run autonomous system (same as run - for clarity)
run-autonomous: all
	@echo "Starting Wall-E autonomous self-balancing robot system..."
	@./wall_e.sh

# Run only arrow detection (for testing)
run-arrows: arrow_detection_process
	@echo "Starting arrow detection process..."
	@./arrow_detection_process

# Run arrow detection with live streaming
run-arrows-stream: arrow_detection_process
	@echo "Starting arrow detection process with live streaming..."
	@echo "Stream available at: http://$(shell hostname -I | awk '{print $$1}'):8080"
	@./arrow_detection_process --stream

# Stop all processes
stop:
	@echo "Stopping Wall-E processes..."
	@pkill -f imu_process || true
	@pkill -f motor_process || true
	@pkill -f control_interface || true
	@pkill -f arrow_detection_process || true
	@pkill -f ultrasonic_process || true
	@pkill -f rpicam-vid || true
	@echo "All processes stopped."

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	rm -f $(TARGETS)
	rm -f *.o
	@echo "Clean complete."

# Debug build with additional symbols
debug: CXXFLAGS += -g -DDEBUG
debug: clean all

# Show system status
status:
	@echo "=== Wall-E System Status ==="
	@echo "Processes running:"
	@ps aux | grep -E "(imu_process|motor_process|control_interface|arrow_detection_process|ultrasonic_process)" | grep -v grep || echo "No Wall-E processes running"
	@echo ""
	@echo "Shared memory:"
	@ls -la /dev/shm/ | grep wall_e || echo "No shared memory segments found"
	@echo ""
	@echo "I2C devices:"
	@ls -la /dev/i2c-* 2>/dev/null || echo "No I2C devices found"
	@echo ""
	@echo "PWM devices:"
	@ls -la /sys/class/pwm/ 2>/dev/null || echo "No PWM devices found"
	@echo ""
	@echo "Camera devices:"
	@ls -la /dev/video* 2>/dev/null || echo "No camera devices found"

# Development help
help:
	@echo "Wall-E Self-Balancing Robot Build System"
	@echo ""
	@echo "Available targets:"
	@echo "  all              - Build all executables"
	@echo "  imu_process      - Build IMU process only"
	@echo "  motor_process    - Build motor process only"
	@echo "  control_interface - Build control interface only"
	@echo "  arrow_detection_process - Build arrow detection process only"
	@echo "  ultrasonic_process - Build ultrasonic sensor process only"
	@echo ""
	@echo "System setup:"
	@echo "  setup            - Complete system setup (permissions, I2C, PWM)"
	@echo "  setup-permissions - Add user to required groups"
	@echo "  setup-i2c        - Enable I2C interface"
	@echo "  setup-pwm        - Enable PWM interface"
	@echo "  check-deps       - Check system dependencies"
	@echo "  test-i2c         - Test I2C connection to MPU6050"
	@echo ""
	@echo "Running:"
	@echo "  run              - Start autonomous system (with arrow detection)"
	@echo "  run-with-arrows  - Start autonomous system (legacy compatibility)"
	@echo "  run-autonomous   - Start autonomous system (same as run)"
	@echo "  run-arrows       - Start arrow detection only (for testing)"
	@echo "  run-arrows-stream - Start arrow detection with live streaming"
	@echo "  stop             - Stop all processes"
	@echo "  status           - Show system status"
	@echo ""
	@echo "Maintenance:"
	@echo "  install          - Install to system directories"
	@echo "  clean            - Remove build artifacts"
	@echo "  debug            - Build with debug symbols"
	@echo "  help             - Show this help"

.PHONY: all install setup setup-permissions setup-i2c setup-pwm check-deps test-i2c run run-with-arrows run-arrows stop clean debug status help
