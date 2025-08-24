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

#### Ultrasonic Sensor (HC-SR04) - Optional
```
HC-SR04         Raspberry Pi 5
VCC        --> 5V (Pin 2 or 4)
GND        --> GND (Pin 6, 9, 14, 20, 25, 30, 34, 39)
Trig       --> GPIO 24 (Pin 18)
Echo       --> GPIO 25 (Pin 22)
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
Wall-E now operates fully autonomously with mode selection:
- **Arrow Detection Mode** - Robot follows arrow directions automatically
- **Arrow-Guided Obstacle Avoidance Scenario** - Robot moves forward, when obstacle detected within 50cm: uses current OR persistent arrow direction to turn 90° (±5°), resets arrow memory after turn, without any arrow guidance: stops scenario
- **Live streaming** - Web browser access to camera feed with overlays
- **System monitoring** - Real-time status display in terminal
- **Emergency handling** - Automatic safety responses
- **Ctrl+C only** - Single command to stop the entire system

#### Mode Selection
When starting the control interface, you can choose:
- **A** - Arrow Detection Mode (default autonomous behavior)
- **S** - Arrow-Guided Obstacle Avoidance Scenario (forward motion with intelligent obstacle avoidance using arrow guidance)
- **Q** - Quit

#### Arrow-Guided Obstacle Avoidance Scenario
The arrow-guided obstacle avoidance scenario demonstrates intelligent navigation:
1. Robot starts in STANDBY mode for balancing stabilization
2. Enables both ultrasonic sensor AND arrow detection systems
3. Transitions to FORWARD mode with moderate speed (45 PWM)
4. Continuously monitors ultrasonic distance readings AND arrow detections
5. **Persistent Arrow Memory**: Remembers last valid arrow direction even when arrow moves out of camera frame
6. When obstacle detected within 50cm:
   - **WITH valid arrow (current OR persistent, LEFT/RIGHT, >70% confidence)**: Turns 90° (±5°) in arrow direction using gyroscope feedback, then resumes forward motion
   - **Arrow direction is RESET after each successful turn**, requiring new arrow detection for next obstacle
   - **WITHOUT any arrow guidance (current or persistent)**: Stops scenario and returns to STANDBY
7. Includes safety timeout (30 seconds maximum) and system health monitoring
8. After completion, returns to normal arrow detection mode

**Arrow Memory System:**
- **Persistent Direction**: Last valid arrow direction is maintained even when arrow disappears from view
- **Reset Trigger**: Arrow direction memory is cleared only after successful 90° turn completion
- **Update Logic**: Memory updates whenever a new valid arrow (>70% confidence) is detected
- **Navigation Continuity**: Enables consistent navigation even with intermittent arrow visibility

**Turn Parameters:**
- Turn speed: 70 PWM for precise control
- Turn angle: 90° ± 5° tolerance using gyroscope position difference measurement
- Valid arrows: LEFT/RIGHT directions with >70% confidence
- Invalid arrows: UP/DOWN/UNKNOWN directions or low confidence
- Safety timeout: 5 seconds maximum per turn attempt
- **Angle Measurement**: Direct gyro_z position difference from start to current position (handles wraparound)

**Safety Features:**
- Automatic timeout protection (30 seconds maximum)
- Real-time system health monitoring (IMU, motors, ultrasonic sensor)
- Emergency stop on component failure
- Moderate speed for safe operation

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
3. Control interface will display mode selection menu:
   - Press **A** for Arrow Detection Mode (default)
   - Press **S** for Obstacle Avoidance Scenario
   - Press **Q** to quit
4. Robot automatically starts balancing and selected mode
5. For Arrow Detection: View live stream in web browser using displayed URL
6. For Scenario: Watch terminal output for obstacle detection progress
7. Use Ctrl+C to stop the entire system

**System Features:**
- **Arrow Detection Mode**: Production use, demonstrations, autonomous navigation
- **Arrow-Guided Obstacle Avoidance Scenario**: Advanced demonstration combining ultrasonic obstacle detection with arrow-based navigation intelligence
- **Live Streaming**: Real-time camera feed with arrow detection overlays (Arrow mode)
- **Safety Monitoring**: Automatic emergency stop on system failures
- **Mode Selection**: Choose between different operational modes at startup

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
   - **Enhanced Computer Vision**: Advanced red arrow detection with improved tip detection
   - **Multi-Factor Analysis**: Combines corner sharpness, positional analysis, isolation scoring, and boundary detection
   - **Side Corner Filtering**: Prevents false detection of arrow side edges as the main tip
   - **Real-time Processing**: Camera feed processing at 10-20 FPS
   - **Accurate Direction Classification**: LEFT, RIGHT, UP, DOWN with improved reliability
   - **Confidence Scoring**: Robust filtering with enhanced validation algorithms
   - **Tip Position Analysis**: Uses relative tip position within bounding box for precise direction determination
   - **Shares Detection Results**: Via shared memory for real-time robot navigation

5. **Ultrasonic Sensor Process** (`ultrasonic_process.cpp`) - *Optional*
   - Distance measurement using HC-SR04 ultrasonic sensor
   - Obstacle detection at 20Hz (50ms cycle time)
   - Moving average filter for noise reduction
   - Real-time distance sharing via shared memory
   - Configurable minimum safe distance (default 20cm)

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

#### Ultrasonic Sensor Fields
- `ultrasonic_distance_cm`: Current distance measurement in centimeters
- `ultrasonic_timestamp`: Last measurement timestamp
- `ultrasonic_active`: Process health status
- `ultrasonic_enabled`: Feature enable/disable flag
- `ultrasonic_min_distance`: Minimum safe distance (configurable)
- `obstacle_detected`: Boolean flag for obstacle within minimum distance

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

## Ultrasonic Sensor System

The Wall-E robot includes an optional ultrasonic distance sensor (HC-SR04) for obstacle detection and avoidance.

### Features
- **Real-time Distance Measurement**: Processes ultrasonic readings at 20Hz (50ms cycle time)
- **Obstacle Detection**: Configurable minimum safe distance (default 20cm)
- **Moving Average Filter**: 5-sample filter for noise reduction and stable readings
- **Range Detection**: Valid range 2-400cm (typical HC-SR04 specifications)
- **Timeout Protection**: Prevents system hang from sensor failures
- **Shared Memory Integration**: Real-time distance sharing with other processes

### Setup for Ultrasonic Sensor

1. **Hardware Connection**: Connect HC-SR04 as shown in wiring diagram above
2. **No Special Configuration**: Works with standard GPIO (no additional kernel modules needed)
3. **Power Requirements**: 5V power supply (connect to Pi's 5V pin)
4. **Test Connection**: Use the test script to verify sensor operation

### Ultrasonic Sensor Usage

```bash
# Build ultrasonic sensor component
make ultrasonic_process

# Test ultrasonic sensor standalone
./test_ultrasonic.sh

# Run full system with ultrasonic sensor (automatic with ./wall_e.sh)
make run

# Individual process testing
./ultrasonic_process  # (requires shared memory from other processes)
```

### Technical Specifications
- **Sensor Model**: HC-SR04 ultrasonic distance sensor
- **Range**: 2cm to 400cm
- **Accuracy**: ±3mm
- **Measurement Angle**: 15 degrees
- **Update Rate**: 20Hz (50ms cycle time)
- **Filter**: 5-sample moving average
- **Timeout**: 10ms trigger timeout, 30ms echo timeout
- **Integration**: Seamless shared memory communication

### Ultrasonic Sensor Parameters
- **Minimum distance**: 20cm (configurable via shared memory)
- **Maximum distance**: 400cm (sensor hardware limit)
- **Measurement frequency**: 20Hz for responsive detection
- **Filter size**: 5 samples for stable readings
- **GPIO pins**: Trigger on GPIO24, Echo on GPIO25

### Live Stream Integration
The ultrasonic sensor data is displayed in the live video stream as:
- **Distance Bar**: Visual bar in top-right corner showing distance (0-100cm range)
- **Color Coding**: 
  - 🟢 Green: Safe distance (>30cm)
  - 🟠 Orange: Close objects (20-30cm)
  - 🔴 Red: Obstacle detected (<20cm)
- **Real-time Updates**: Bar updates at 20Hz with distance measurements
- **Text Display**: Numerical distance value and obstacle alerts

### Performance
- **CPU Usage**: ~2-3% on Raspberry Pi 5
- **Memory**: ~5MB usage
- **Latency**: ~50ms measurement cycle
- **Integration**: Real-time shared memory updates
- **Reliability**: Timeout protection and error handling

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
  - **Ultrasonic distance bar** (when ultrasonic sensor is active)
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
- **Console Output**: Silent operation - no detection logging, all feedback in live stream

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

### Ultrasonic Sensor Issues

10. **"Ultrasonic process failed to start"**
    - Check GPIO24 and GPIO25 connections
    - Verify 5V power supply to sensor
    - Test with standalone script: `./test_ultrasonic.sh`
    - Ensure user is in `gpio` group

11. **"Timeout waiting for echo" or invalid readings**
    - Check wiring: Trig to GPIO24, Echo to GPIO25
    - Verify 5V power connection (VCC pin)
    - Ensure GND connection is secure
    - Check for loose connections or damaged sensor
    - Test in open space (no immediate obstacles)

12. **Distance readings seem inaccurate**
    - Calibrate by testing known distances
    - Check for interference from other ultrasonic devices
    - Ensure sensor is mounted firmly (vibration affects readings)
    - Verify temperature compensation (sound speed varies with temperature)
    - Test different angles and surfaces (hard surfaces work best)

13. **Occasional timeout errors in ultrasonic readings**
    - These are normal and expected (~0.1-1% of readings)
    - Caused by environmental factors (air currents, soft surfaces, angles)
    - The system uses a moving average filter to handle these
    - Only concerned if error rate exceeds 5-10%
    - Check connections if errors become frequent

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

## Summary of Ultrasonic Sensor Integration

The ultrasonic sensor application has been successfully created and integrated into your existing Wall-E self-balancing robot system. Here's what was implemented:

### ✅ What Was Created

1. **`ultrasonic_process.cpp`** - Complete ultrasonic sensor application with:
   - HC-SR04 sensor control via GPIO24 (Trig) and GPIO25 (Echo)
   - Raspberry Pi 5 GPIO detection and mapping
   - 20Hz measurement cycle (50ms) for responsive obstacle detection
   - Moving average filter (5 samples) for stable readings
   - Timeout protection for reliable operation
   - Shared memory integration with other processes

2. **Shared Memory Integration** - Extended `shared_memory.h` with:
   - `ultrasonic_distance_cm` - Current distance in centimeters
   - `ultrasonic_timestamp` - Last measurement timestamp
   - `ultrasonic_active` - Process health monitoring
   - `ultrasonic_enabled` - Feature enable/disable control
   - `ultrasonic_min_distance` - Configurable safe distance (default 20cm)
   - `obstacle_detected` - Boolean flag for obstacle detection

3. **Build System Updates** - Modified `Makefile` to include:
   - Ultrasonic process compilation target
   - Installation and cleanup commands
   - Status monitoring and help documentation

4. **System Integration** - Updated `wall_e.sh` to:
   - Automatically start ultrasonic process with the main system
   - Include process monitoring and cleanup
   - Display ultrasonic sensor status in system information

5. **Control Interface** - Enhanced `control_interface.cpp` to:
   - Display real-time distance measurements
   - Show obstacle detection status
   - Monitor ultrasonic process health

6. **Test Script** - Created `test_ultrasonic.sh` for:
   - Independent sensor testing
   - Connection verification
   - Troubleshooting guidance

### 🔧 How to Use

1. **Hardware Setup**: Connect HC-SR04 sensor:
   - VCC → 5V, GND → Ground
   - Trig → GPIO24, Echo → GPIO25

2. **Test the Sensor**: 
   ```bash
   ./test_ultrasonic.sh
   ```

3. **Run with Full System**:
   ```bash
   ./wall_e.sh  # Automatically includes ultrasonic sensor
   ```

4. **Build Individual Process**:
   ```bash
   make ultrasonic_process
   ```

### 📊 System Features

- **Real-time Distance**: 20Hz updates for responsive obstacle detection
- **Noise Filtering**: Moving average for stable measurements
- **Range**: 2-400cm measurement range with 3mm accuracy
- **Safety Integration**: Configurable minimum safe distance
- **Process Health**: Automatic monitoring and error handling
- **Pi 5 Compatible**: Full GPIO offset detection for Raspberry Pi 5
- **Live Stream Integration**: Visual distance bar overlay in camera feed
- **Silent Operation**: No console debug output, all data visible in video overlay

The ultrasonic sensor seamlessly integrates with your existing arrow detection, IMU, and motor control systems through the shared memory architecture, providing real-time obstacle detection capabilities to your self-balancing robot.

---

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
