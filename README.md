# SpotMicro Robot Dog Control System

A comprehensive control system for the SpotMicro robot dog featuring web interface, autonomous behaviors, and environmental monitoring.

## Features

### 1. Web Control Interface
- **Responsive Web UI**: Beautiful, modern control panel accessible via browser
- **Movement Controls**: Forward, backward, left, right, turn left/right
- **Posture Controls**: Sit, stand, and other poses
- **Actions**: Give paw, take photos
- **Keyboard Support**: Control the robot using arrow keys and WASD

### 2. Autonomous Behaviors

#### Obstacle Avoidance
- Uses dual ultrasonic sensors (left and right)
- Automatically detects obstacles <30cm while moving forward
- Turns toward the side with more space
- Resumes forward movement when path is clear

#### Touch Sensor Response
- **While walking**: Automatically stops when touched
- **While sitting**: Gives paw when touched
- **While standing**: Sits down and gives paw

### 3. Environment Monitoring
- **Ultrasonic Sensors**: Measures distances on left and right sides
- **Touch Sensor**: Detects physical contact
- **IMU Data**: Tracks orientation (roll and pitch)
- **CSV Logging**: Records all sensor data and commands to `rl_environment_log.csv`

### 4. Camera Integration
- Capture photos on command using Raspberry Pi Camera
- Images saved with timestamps: `capture_<timestamp>.jpg`

## Hardware Configuration

### GPIO Pins (BCM Mode)
- **Left Ultrasonic Sensor**: TRIG=14, ECHO=15
- **Right Ultrasonic Sensor**: TRIG=23, ECHO=24
- **Touch Sensor**: Pin 17 (with pull-up resistor)

### Required Hardware
- Raspberry Pi (with camera module)
- 2x HC-SR04 Ultrasonic Sensors
- Touch/button sensor
- SpotMicro servo setup (as per original design)

## Software Requirements

```bash
# Core dependencies (for robot)
pip install RPi.GPIO
pip install picamera2
pip install opencv-python
pip install numpy
pip install pygame
pip install adafruit-circuitpython-pca9685
pip install adafruit-circuitpython-mpu6050
pip install adafruit-circuitpython-servokit

# Web interface (for app)
pip install Flask
```

## Usage

### Starting the Robot Controller

```bash
python3 oop.py
```

This starts:
- The main robot control loop
- Socket server on `localhost:5055`
- Console interface for direct commands
- Sensor monitoring and logging

### Starting the Web Control Panel

```bash
python3 app.py
```

Then open your browser to: `http://localhost:5000`

### Console Commands

Available commands when running `oop.py`:
- **Movement**: `forward`, `backward`, `left`, `right`, `turn_left`, `turn_right`
- **Walking**: `walk`, `stop_walk`
- **Postures**: `sit`, `stand`, `lie`
- **Actions**: `paw_left`, `paw_right`, `paw_down`, `photo`
- **Settings**: `move`, `anim`, `trot`, `imu`
- **Exit**: `quit`

### Web Interface Controls

The web interface provides:
- Direction pad for movement
- Turn left/right buttons
- Stop button
- Sit/Stand buttons
- Give Paw action
- Take Photo button

Keyboard shortcuts are also available:
- Arrow keys / WASD: Movement
- Q/E: Turn left/right
- Space: Stop

## Architecture

### Communication Flow
```
Web Browser (port 5000)
    ↓ HTTP
Flask App (app.py)
    ↓ Socket (port 5055)
Robot Controller (oop.py)
    ↓
Servos, Sensors, Camera
```

### Data Logging

Environment data is logged to `rl_environment_log.csv` with the following fields:
- `timestamp`: Unix timestamp
- `left_distance`: Left ultrasonic sensor reading (cm)
- `right_distance`: Right ultrasonic sensor reading (cm)
- `touch_state`: Touch sensor state (0=not pressed, 1=pressed)
- `imu_roll`: IMU roll angle
- `imu_pitch`: IMU pitch angle
- `current_command`: Current movement command

## Safety Features

- **Obstacle Avoidance**: Prevents collisions during forward movement
- **Touch Response**: Immediate reaction to touch sensor
- **Timeout Protection**: Ultrasonic sensors have timeout mechanisms
- **Graceful Degradation**: System works even if some sensors are unavailable
- **Secure Web Interface**: Flask runs in production mode (debug disabled)

## Development and Testing

Run integration tests:
```bash
python3 test_integration.py
```

This verifies:
- Module imports
- Flask routes
- Required methods
- Socket functionality
- CSV logging
- Template files

## File Structure

```
pes/
├── oop.py                    # Main robot controller
├── app.py                    # Flask web application
├── templates/
│   └── index.html           # Web control interface
├── test_integration.py      # Integration tests
├── .gitignore              # Git ignore rules
├── rl_environment_log.csv  # Environment data log (auto-generated)
└── capture_*.jpg           # Photos (auto-generated)
```

## Troubleshooting

### No module named 'RPi.GPIO'
This is normal on non-Raspberry Pi systems. The code includes graceful fallbacks.

### Socket connection refused
Make sure `oop.py` is running and the socket server initialized on port 5055.

### Camera not working
Ensure:
- Raspberry Pi camera is connected and enabled
- `picamera2` and `cv2` are installed
- Camera interface is enabled in `raspi-config`

### Sensors returning invalid values
Check:
- GPIO connections are correct
- Sensors are powered properly
- No loose wires

## Contributing

When making changes:
1. Run syntax validation: `python3 -m py_compile <file>.py`
2. Run integration tests: `python3 test_integration.py`
3. Test on actual hardware when possible

## License

See repository root for license information.
