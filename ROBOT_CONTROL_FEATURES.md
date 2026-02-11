# Robot Control Enhancements

This document describes the new features added to the SpotMicro robot controller.

## New Features

### 1. Sensor Integration

#### HC-SR04 Ultrasonic Sensors
- **Left Sensor**: GPIO 14 (TRIG), GPIO 15 (ECHO)
- **Right Sensor**: GPIO 23 (TRIG), GPIO 24 (ECHO)
- Measures distances from 2 to 400 cm
- Automatically reads sensors every 10 frames

#### Touch Sensor
- **Pin**: GPIO 17
- Pull-up configuration
- Implements touch reaction sequence

#### Obstacle Avoidance
When moving forward:
- Detects obstacles within 30 cm
- Automatically turns towards the clearer side
- Left blocked → turns right
- Right blocked → turns left
- Both blocked → turns towards less blocked side

#### Touch Reaction Sequence
1. First touch: **Stop** all movements
2. Second touch: **Sit** down
3. Third touch (and subsequent): **Give Paw (Right)**

### 2. Environment Logging (Reinforcement Learning)

#### CSV Log File
- **Location**: `rl_environment_log.csv`
- Logs every 30 frames (approximately 2 times per second at 60 FPS)

#### Logged Data
- Timestamp
- Current command
- Current action/state
- Left ultrasonic distance
- Right ultrasonic distance
- Touch sensor state
- IMU data (roll, pitch, accelerometer X/Y/Z)
- All 12 servo angles

### 3. TCP Server for App Control

#### Server Configuration
- **Port**: 5000
- **Protocol**: TCP
- **Binding**: 0.0.0.0 (all interfaces)
- Multi-client support with threading

#### Supported Commands
All standard robot commands:
- Movement: `forward`, `backward`, `left`, `right`, `turn_left`, `turn_right`
- Actions: `sit`, `stand`, `lie`, `twist`, `pee`, `stop`, `walk`
- Pawing: `paw_left`, `paw_right`, `paw_down`
- Camera: `photo`
- Settings: `move`, `anim`, `trot`, `imu`
- Control: `quit`

#### Usage Example
```python
import socket

# Connect to robot
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('ROBOT_IP', 5000))

# Send command
sock.send(b'forward\n')
response = sock.recv(1024)
print(response)  # b'OK'

# Take a photo
sock.send(b'photo\n')
response = sock.recv(1024)
print(response)  # b'OK' or b'ERROR'

sock.close()
```

### 4. Camera Integration

#### Hardware
- Uses **Picamera2** library
- Processes images with **OpenCV**

#### Photo Capture
- Command: `photo` (console or TCP)
- Photos saved to: `/home/runner/work/pes/pes/photos/`
- Filename format: `photo_YYYYMMDD_HHMMSS.jpg`
- Images in BGR format (OpenCV standard)

#### Features
- Automatic camera initialization
- 0.5s warm-up time
- Graceful degradation if camera unavailable

### 5. Code Refactoring

#### Import Fix
`work_with_voice.py` now correctly imports from `oop.py`:
```python
from oop import SpotMicroController  # Fixed from oop3
```

## Implementation Details

### GPIO Configuration
```python
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
```

### Sensor Reading
Sensors are read every 10 frames to balance responsiveness and CPU usage:
```python
if self.frame_counter % 10 == 0:
    self.read_sensors()
    if self.walking:
        self.handle_obstacle_avoidance()
```

### Logging
State is logged every 30 frames:
```python
if self.frame_counter % 30 == 0:
    self.log_state()
```

### Resource Cleanup
Proper cleanup on exit:
- Closes TCP server socket
- Stops camera
- Cleans up GPIO pins

## Safety Notes

1. **Obstacle Avoidance**: Only active when moving forward
2. **Sensor Filtering**: Invalid readings (< 2cm or > 400cm) are ignored
3. **Touch Debouncing**: State transitions prevent rapid toggling
4. **TCP Timeout**: Connections timeout after 30 seconds of inactivity

## Dependencies

New dependencies added:
- `csv` (built-in)
- `socket` (built-in)
- `datetime` (built-in)
- `RPi.GPIO` (already required)
- `picamera2` (optional, for camera)
- `opencv-python` (optional, for camera)

## Testing

Run the functionality test:
```bash
python3 /tmp/test_oop_functionality.py
```

## Future Enhancements

Potential improvements:
1. Configurable obstacle detection threshold
2. Multiple touch sequences
3. Image processing for object detection
4. WebSocket support for real-time streaming
5. MQTT integration for IoT platforms
