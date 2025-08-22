# Wall-E Self-Balancing Robot

A two-wheeled self-balancing robot implementation for Raspberry Pi 5. This implementation uses shared memory for high-speed communication between the IMU sensor process and motor control process, achieving real-time 5ms control cycles.

## Features

- **Real-time balancing control** with 5ms cycle time (200Hz)
- **Shared memory communication** for minimal latency between processes
- **Kalman filter** for sensor fusion and precise angle estimation
- **Arrow detection system** with computer vision for navigation guidance
- **Emergency stop handling** with automatic recovery
- **Interactive control interface** with keyboard commands
- **Hardware compatibility** for Raspberry Pi 5 (adaptable to other Pi models)
- **Safety features** including angle limits and emergency override

## Hardware Requirements

### Components
- Raspberry Pi 5 (or compatible model)
- MPU6050 IMU sensor (6-axis accelerometer/gyroscope)
- TB6612FNG motor driver (or compatible dual motor driver)
- 2x DC motors with encoders
- Robot chassis (two-wheeled configuration)
- Battery pack (7.4V Li-Po recommended)
- **Raspberry Pi Camera Module** (for arrow detection - optional)

### Connections

#### MPU6050 (I2C)
```
MPU6050    Raspberry Pi 5
VCC    --> 3.3V (Pin 1)
GND    --> GND (Pin 6)
SDA    --> GPIO 2 (Pin 3) - I2C1 SDA
SCL    --> GPIO 3 (Pin 5) - I2C1 SCL
```

#### Motor Driver (TB6612FNG)
```
Motor Driver    Raspberry Pi 5
STBY       --> GPIO 20 (Pin 38)
AIN1       --> GPIO 26 (Pin 37) - Left motor direction
AIN2       --> GND
PWMA       --> GPIO 12 (Pin 32) - Left motor PWM
BIN1       --> GPIO 21 (Pin 40) - Right motor direction  
BIN2       --> GND
PWMB       --> GPIO 13 (Pin 33) - Right motor PWM
VM         --> Battery positive (7.4V)
VCC        --> 3.3V (Pin 17)
GND        --> GND (Pin 20, 34, 39)
```

#### Encoders
```
Left Encoder    --> GPIO 17 (Pin 11)
Right Encoder   --> GPIO 27 (Pin 13)
```

#### Camera Module (Optional - for Arrow Detection)
```
Raspberry Pi Camera Module V2/V3
Connect to camera port on Raspberry Pi 5
```

## Software Setup

### Prerequisites

1. **Enable I2C and PWM**:
```bash
sudo raspi-config
# Navigate to Interface Options > I2C > Yes
# Navigate to Interface Options > Camera > Yes (for arrow detection)
# Add to /boot/config.txt:
# dtparam=i2c_arm=on
# dtparam=pwm=on
```

2. **Install dependencies**:
```bash
sudo apt update
sudo apt install build-essential i2c-tools libi2c-dev git

# For arrow detection (optional)
sudo apt install libopencv-dev cmake pkg-config
sudo apt install rpicam-apps  # Usually pre-installed on Pi OS
```

3. **Add user to required groups**:
```bash
sudo usermod -a -G gpio,i2c,dialout $USER
# Log out and back in for changes to take effect
```

### Building the Project

1. **Clone and navigate to the project**:
```bash
cd /home/shiko/build/tumbller/Wall_E
```

2. **Check system dependencies**:
```bash
make check-deps
```

3. **Complete system setup** (first time only):
```bash
make setup
```

4. **Build all executables**:
```bash
make all
```

5. **Test I2C connection**:
```bash
make test-i2c
```

## Running the System

## Running the System

### Autonomous Mode (Default)
```bash
# Start the autonomous system
./wall_e.sh

# OR use make command
make run
```

The system now operates autonomously by default with:
- Automatic arrow detection and following
- Live streaming with visual overlays  
- No manual controls (only Ctrl+C to stop)
- Real-time system monitoring

### Individual Process Control

### Using Make Commands
```bash
# Start autonomous system (default)
make run

# Start autonomous system (alternative commands)
make run-with-arrows
make run-autonomous

# Start arrow detection only (for testing)
make run-arrows

# Stop all processes
make stop

# Check system status
make status
```

## Control Commands

### Autonomous Mode (Default)
Wall-E now operates fully autonomously:
- **No manual controls** - Robot follows arrow directions automatically
- **Live streaming** - Web browser access to camera feed with overlays
- **System monitoring** - Real-time status display in terminal
- **Emergency handling** - Automatic safety responses
- **Ctrl+C only** - Single command to stop the entire system

The interface displays:
- Balance angle and motion status
- Arrow detection results with confidence
- System health (IMU, motors, camera)
- PWM values and speed settings
- Emergency and safety status

### Control Flow

#### Autonomous Operation
1. Start the system with `./wall_e.sh` or `make run`
2. Wait for "AUTONOMOUS SYSTEM READY!" message
3. Robot automatically starts balancing and arrow detection
4. View live stream in web browser using displayed URL
5. Robot follows detected arrows automatically
6. Use Ctrl+C to stop the entire system

**System Features:**
- **Autonomous Mode**: Production use, demonstrations, autonomous operation
- **Live Streaming**: Real-time camera feed with arrow detection overlays
- **Safety Monitoring**: Automatic emergency stop on system failures
- **No User Input**: Prevents accidental commands during operation

## Technical Details

### Architecture

The system consists of four main processes:

1. **IMU Process** (`imu_process.cpp`)
   - Reads MPU6050 sensor data at 200Hz
   - Applies Kalman filter for angle estimation
   - Monitors emergency stop conditions
   - Updates shared memory with sensor data

2. **Motor Process** (`motor_process.cpp`)
   - Reads control data from shared memory at 200Hz
   - Implements PID balance control for stability
   - Controls motor PWM and direction
   - Handles encoder feedback

3. **Control Interface** (`control_interface.cpp`)
   - Provides user interaction
   - Updates motion commands in shared memory
   - Displays real-time system status
   - Handles safety controls

4. **Arrow Detection Process** (`arrow_detection_process.cpp`) - *Optional*
   - Computer vision-based red arrow detection
   - Real-time camera feed processing at 10-20 FPS
   - Direction classification (LEFT, RIGHT, UP, DOWN)
   - Confidence scoring and robust filtering
   - Shares detection results via shared memory

### Shared Memory Structure

The `BalanceData` structure contains:
- Filtered IMU angles and angular velocities
- Motor PWM values and encoder counts
- Motion mode and system state flags
- PID control variables and parameters
- Safety and emergency stop flags
- **Arrow detection data** (direction, confidence, timestamps)

#### Arrow Detection Fields
- `detected_arrow_direction`: Current detected arrow direction
- `arrow_confidence`: Detection confidence percentage (0-100%)
- `arrow_timestamp`: Last detection timestamp
- `arrow_detection_active`: Process health status
- `arrow_detection_enabled`: Feature enable/disable flag

### Control Parameters

The system uses optimized PID parameters:
- **Balance PID**: Kp=55.0, Kd=0.75
- **Speed PID**: Kp=10.0, Ki=0.26  
- **Turn PID**: Kp=2.5, Kd=0.5
- **Angle Limits**: ±22° for emergency stop
- **Cycle Time**: 5ms (200Hz) for real-time response

### Safety Features

- **Angle monitoring**: Automatic emergency stop if robot tilts beyond ±22°
- **Process monitoring**: System stops if any process fails
- **Manual emergency stop**: Immediate motor shutdown with key '4'
- **Safety override**: Manual disable of automatic safety features
- **Graceful shutdown**: Clean process termination with Ctrl+C

## Arrow Detection System

The Wall-E robot includes an optional computer vision system for detecting red arrows with live streaming capabilities for navigation guidance.

### Features
- **Real-time Detection**: Processes camera feed at 10-20 FPS
- **Live HTTP Streaming**: MJPEG stream viewable in web browsers
- **Red Arrow Recognition**: Specifically tuned for red-colored arrows
- **Direction Classification**: Detects arrows pointing LEFT, RIGHT, UP, DOWN
- **Confidence Scoring**: Provides confidence percentage for each detection
- **Visual Overlays**: Bounding boxes, direction labels, confidence scores in stream
- **Robust Filtering**: Advanced contour processing to avoid false positives

### Setup for Arrow Detection

1. **Camera Connection**: Connect Raspberry Pi Camera Module to camera port
2. **Enable Camera**: `sudo raspi-config` → Interface Options → Camera → Yes
3. **Build with OpenCV**: Ensure OpenCV dependencies are installed (see prerequisites)
4. **Test Camera**: `rpicam-hello --list-cameras`

### Arrow Detection Usage

```bash
# Build arrow detection component
make arrow_detection_process

# Test arrow detection standalone with streaming
make run-arrows

# Run full system with arrow detection and streaming
make run-with-arrows

# OR use the shell scripts:
./wall_e.sh arrows
# OR
./wall_e_with_arrows.sh

# View live stream with overlays
# The system automatically detects and displays your Pi's IP address
# Look for the "WALL-E LIVE CAMERA STREAM" box in the terminal output
# URL format: http://YOUR_DETECTED_IP:8080

# In control interface:
# Press 'a' to toggle arrow detection
# Press '6' to set function mode to ARROW_DETECTION
```

### Live Streaming Features
- **Automatic IP Detection**: System detects and displays your Pi's network address
- **Port**: Default 8080 (accessible via web browser)
- **Format**: MJPEG streaming compatible with all browsers
- **Easy Access**: Clear startup message shows exact URL to open
- **Overlays**: Real-time visual feedback showing:
  - Detected arrow bounding boxes
  - Direction labels and confidence percentages
  - Contour outlines for detected shapes
  - System status (balance angle, emergency state)
  - Frame count and processing statistics

### Arrow Detection Parameters
- **Minimum arrow size**: 2% of camera frame
- **Minimum confidence**: 70% for valid detection  
- **Camera resolution**: 640x480 optimized for processing speed
- **Detection range**: Works best with arrows 2-95% of frame size
- **Color detection**: HSV-based red color filtering
- **Streaming latency**: ~100-200ms for live viewing

### Performance
- **CPU Usage**: ~15-20% additional on Raspberry Pi 5 (including streaming)
- **Memory**: ~70MB additional usage (including streaming buffers)
- **Latency**: ~50ms detection latency + ~100ms streaming latency
- **Integration**: Seamless with existing shared memory system
- **Network**: Minimal bandwidth usage, optimized for local viewing

## Troubleshooting

### Common Issues

1. **"Failed to open I2C bus"**
   - Check I2C is enabled: `sudo raspi-config`
   - Verify device exists: `ls /dev/i2c-*`
   - Check permissions: User must be in `i2c` group

2. **"MPU6050 not detected"**
   - Check wiring connections
   - Test with: `make test-i2c`
   - Verify address 0x68 appears in i2cdetect output

3. **"Failed to initialize PWM"**
   - Add `dtparam=pwm=on` to `/boot/config.txt`
   - Reboot after making changes
   - Check: `ls /sys/class/pwm/`

4. **"Failed to setup GPIO"**
   - Add user to `gpio` group: `sudo usermod -a -G gpio $USER`
   - Log out and back in
   - Try running with `sudo` (not recommended for normal use)

5. **Robot doesn't balance**
   - Check motor connections and polarity
   - Verify encoder connections
   - Ensure robot starts in upright position
   - Check for mechanical issues (friction, alignment)

### Arrow Detection Issues

6. **"Failed to initialize rpicam video stream"**
   - Check camera connection and ribbon cable
   - Enable camera: `sudo raspi-config` → Interface Options → Camera
   - Test camera: `rpicam-hello --list-cameras`
   - Verify rpicam tools work: `rpicam-still -o test.jpg`

7. **"OpenCV not found"**
   - Install OpenCV: `sudo apt install libopencv-dev`
   - Check installation: `pkg-config --exists opencv4`

8. **Poor arrow detection accuracy**
   - Ensure good lighting conditions
   - Use clearly red-colored arrows (bright red works best)
   - Check arrow size (should be at least 2% of camera view)
   - Avoid red objects in background
   - Test with: `make run-arrows`

9. **Live streaming not working**
   - Check if port 8080 is available: `netstat -tuln | grep 8080`
   - Verify firewall allows port 8080: `sudo ufw allow 8080`
   - Test local access: `curl http://localhost:8080`
   - Ensure camera is working: `rpicam-still -o test.jpg`
   - Check network connectivity to Pi from viewing device

### Debug Mode

Build with debug symbols:
```bash
make debug
```

Monitor system processes:
```bash
make status
```

Check shared memory:
```bash
ls -la /dev/shm/wall_e_balance
```

### Log Analysis

Each process outputs status information:
- IMU process: Angle readings and emergency status
- Motor process: PWM values and motion mode
- Control interface: Real-time system status

## Performance Tuning

### PID Tuning
Modify the constants in `shared_memory.h`:
```cpp
#define KP_BALANCE 55.0f    // Increase for more aggressive balance correction
#define KD_BALANCE 0.75f    // Increase to reduce oscillation
#define KP_SPEED 10.0f      // Increase for faster speed response
#define KI_SPEED 0.26f      // Increase to reduce steady-state error
```

### Timing Adjustments
The 5ms cycle time can be modified in `shared_memory.h`:
```cpp
#define CYCLE_TIME_MS 5     // Decrease for faster response (higher CPU usage)
```

### Kalman Filter Tuning
Adjust filter parameters in `imu_process.cpp`:
```cpp
const float Q_angle = 0.001f;   // Process noise (lower = trust gyro more)
const float Q_gyro = 0.005f;    // Gyro noise
const float R_angle = 0.5f;     // Measurement noise (lower = trust accel more)
```

## License

This project is provided for educational and personal use.

## Contributing

Feel free to submit issues and enhancement requests. When contributing:

1. Test thoroughly on actual hardware
2. Maintain compatibility with the existing shared memory interface
3. Follow the existing code style and documentation standards
4. Include safety considerations in any modifications

## Acknowledgments

- Raspberry Pi Foundation for the GPIO and I2C interfaces
- MPU6050 community for sensor integration examples

---

**⚠️ Safety Warning**: This robot contains moving parts and can move unexpectedly. Always supervise operation, keep clear of pinch points, and be ready to use emergency stop features. Test in a safe environment away from people and obstacles.
