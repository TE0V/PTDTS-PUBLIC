# PTDTS System Architecture

## Overview
Pan-Tilt Drone Detection and Tracking System (PTDTS) is a multi-modal drone detection and tracking system that combines acoustic detection, visual identification, and precision tracking.

## System States

```
LISTENING  → Acoustic monitoring for drone signatures
    ↓ (Drone detected at azimuth θ)
PANNING    → Fast slew to acoustic bearing, decelerate near target
    ↓ (Arrived at position + visual scan)
DETECTING  → HQ camera searches for visual confirmation
    ↓ (YOLO confirms drone)
TRACKING   → Continuous velocity-based tracking with dual cameras
    ↓ (Target lost)
LISTENING  → Return to acoustic monitoring
```

## Hardware Components

### Compute
- Raspberry Pi 5 (16GB RAM)
- 512GB NVMe SSD
- Active cooling

### Pan Axis (DC Motor)
- Pololu 30:1 Metal Gearmotor with 64 CPR encoder
- External 4.5:1 gear reduction (24T to 108T)
- Total reduction: 135:1
- DRV8874 motor driver (12V rail)
- LS7366R encoder buffer (SPI interface)

### Tilt Axis (Servo)
- Axon MAX MK2 servo (6V rail)
- Analog position feedback via ADS1115 ADC (I2C)
- PWM control signal (GPIO 18 - hardware PWM required for Pi 5)

### Cameras
- Camera 0 (CAM0): Raspberry Pi HQ Camera (Sony IMX477)
  - Detection mode: 1280x720 @ 30 FPS
  - Wide field of view
- Camera 1 (CAM1): Raspberry Pi Global Shutter Camera (Sony IMX296)
  - Tracking mode: 640x480 @ 60 FPS
  - Reduced motion blur
- Baseline: 50mm (side-by-side, over-under configuration)

### Acoustic Detection
- ReSpeaker XMOS XVF3800 (4-microphone array)
- Circular geometry: 46.5mm radius
- USB audio interface
- ODAS (Open embeddeD Audition System) for DOA

### Power System
- LRS-150 24V Power Supply (150W)
- 5V Rail: Pololu D36V50F5 (Raspberry Pi)
- 6V Rail: Pololu D36V50F6 (Servo)
- 12V Rail: Pololu D36V50F12 (Motor Driver)

## GPIO Pinout Reference

### Power Rails
- Pin 1 (3.3V): ADS1115 VCC
- Pin 2 (5V): Pi power from step-down converter
- Pin 4 (5V): LS7366R board power
- Pin 17 (3.3V): DRV8874 PMODE + SLEEP (always enabled)

### Pan Motor (DRV8874)
- Pin 13 (GPIO 27): DRV8874 IN1
- Pin 15 (GPIO 22): DRV8874 IN2
- Pin 14 (GND): DRV8874 GND

### Encoder (LS7366R via SPI0)
- Pin 19 (GPIO 10): LS7366R MOSI (SPI0)
- Pin 21 (GPIO 9): LS7366R MISO (SPI0)
- Pin 23 (GPIO 11): LS7366R SCK (SPI0)
- Pin 24 (GPIO 8): LS7366R CS1 (SPI0)
- Pin 4 (5V): LS7366R board power
- Pin 9 (GND): LS7366R board ground

### Tilt Servo & Position Feedback
- Pin 12 (GPIO 18): Servo SIGNAL (PWM1 - hardware PWM)
- Pin 20 (GND): Servo GND + stepdown GND
- Pin 3 (GPIO 2): I2C SDA (ADS1115)
- Pin 5 (GPIO 3): I2C SCL (ADS1115)
- Pin 1 (3.3V): ADS1115 VCC
- Pin 25 (GND): ADS1115 GND

### Free GPIO Pins
- Pin 7 (GPIO 4): FREE (was Encoder B)
- Pin 11 (GPIO 17): FREE (was Encoder A)
- Pin 22 (GPIO 25): FREE (was Servo - software PWM doesn't work on Pi 5)

### Ground Connections
- Pin 6 (GND): Pi ground from step-down
- Pin 9 (GND): LS7366R board ground
- Pin 14 (GND): DRV8874 GND
- Pin 20 (GND): Servo GND + stepdown GND
- Pin 25 (GND): ADS1115 GND

## Software Architecture

### Layer 1: Hardware Abstraction Layer (HAL)
All hardware interfaces with simulation mode support:

```
EncoderInterface
├── LS7366REncoder (production)
└── SimulatedEncoder (testing)

MotorInterface
├── DRV8874Motor (production)
└── SimulatedMotor (testing)

ServoInterface
├── AxonServo (production - PWM only)
├── AxonServoClosedLoop (production - with ADS1115 feedback)
└── SimulatedServo (testing)

CameraInterface
├── Picamera2Camera (production)
└── SimulatedCamera (testing)

AudioInterface
├── ODASAcousticDetector (production)
└── SimulatedAcoustic (testing)
```

### Layer 2: Control Systems

**PanController**
- Position-based control for initial acoustic slew
- Velocity-based PID for continuous tracking
- Trapezoidal velocity profile for smooth motion
- Fixed 50Hz control loop

**TiltController**
- Position-based control only
- Smooth interpolation
- Optional closed-loop with ADS1115 feedback

**TrackingController**
- Pixel error → target velocity conversion
- PID velocity control (Kp, Ki, Kd all active)
- Anti-windup integral clamping
- Deadzone for jitter prevention

### Layer 3: Detection Systems

**AcousticDetector**
- ODAS socket listener (port 9000)
- Direction of arrival (DOA) extraction
- Bandpass filter: 100-500 Hz (drone frequencies)
- Energy threshold: configurable
- Output: Azimuth (0-360°), confidence

**VisualDetector**
- YOLO11n UAV-finetune NCNN model
- Multi-camera support (HQ for detection, GS for tracking)
- Configurable confidence thresholds
- Output: Bounding box, center pixel, class, confidence

### Layer 4: State Machine

**States:**
- LISTENING: Acoustic monitoring, cameras idle
- PANNING: Fast slew to acoustic bearing with deceleration
- DETECTING: HQ camera visual search at acoustic location
- TRACKING: Continuous velocity tracking with dual cameras
- MANUAL: User control via Xbox controller or keyboard
- ERROR: Hardware fault recovery

**State Transitions:**
```python
LISTENING + acoustic_detection → PANNING
PANNING + arrived_at_target → DETECTING
DETECTING + visual_confirmation → TRACKING
TRACKING + target_lost → LISTENING
ANY + manual_control → MANUAL
MANUAL + auto_mode → LISTENING
ANY + hardware_fault → ERROR
```

### Layer 5: Coordinate Systems

**World Coordinate System: ENU (East-North-Up)**
- X-axis: East (positive = eastward)
- Y-axis: North (positive = northward)
- Z-axis: Up (positive = upward)
- Origin: System mounting point

**Pan Angle (Azimuth)**
- 0° = North
- 90° = East
- 180° = South
- 270° = West
- Encoder counts → degrees conversion using calibrated CPR

**Tilt Angle (Elevation)**
- Servo convention: 90° = horizontal
- 0° = looking down
- 180° = looking up

**Pixel → Angle Conversion:**
```
pixel_offset_h = (pixel_x - frame_width/2) / frame_width * fov_h
pixel_offset_v = (pixel_y - frame_height/2) / frame_height * fov_v
azimuth = pan_angle + pixel_offset_h
elevation = tilt_angle + pixel_offset_v
```

**3D Position Estimation (with range):**
```
x = range * cos(elevation) * sin(azimuth)  # East
y = range * cos(elevation) * cos(azimuth)  # North
z = range * sin(elevation)                 # Up
```

### Layer 6: Data Pipeline

**Telemetry Data:**
- Pan position, velocity, motor PWM
- Tilt position, servo feedback
- Camera FPS, detection count
- Target position, velocity, confidence
- System state, uptime, errors

**Data Logging:**
- Timestamped CSV for post-analysis
- Real-time WebSocket streaming
- Configurable log levels
- Rotating file handlers

**2D Map View:**
- Top-down projection (X-Y plane)
- Target trajectory history
- System position at origin
- Real-time position updates

## Control Flow

### Main Thread (50Hz)
```python
while running:
    # Read sensors
    pan_angle = encoder.get_angle()
    tilt_angle = servo.get_position()

    # State machine update
    state_machine.update()

    # Control output
    if state == TRACKING:
        motor_command = tracking_controller.compute()
        motor.set_velocity(motor_command)
    elif state == PANNING:
        position_controller.move_to(target_angle)

    # Telemetry
    log_data()
    broadcast_telemetry()

    sleep(0.02)  # 50Hz
```

### Detection Thread (30-60Hz)
```python
while running:
    frame = camera.capture()
    detections = yolo.detect(frame)

    if detections:
        best = max(detections, key=lambda d: d.confidence)
        tracking_controller.update_target(best.center_x)
    else:
        tracking_controller.clear_target()

    annotated_frame = draw_overlays(frame, detections)
    web_streamer.update_frame(annotated_frame)
```

### Acoustic Thread (Continuous)
```python
while running:
    data = odas_socket.receive()
    sources = parse_odas_json(data)

    for source in sources:
        if is_drone_signature(source):
            azimuth = source['azimuth']
            state_machine.queue_acoustic_detection(azimuth)
```

### Web Server Thread
```python
# Flask routes
@app.route('/api/status')
def get_status():
    return jsonify(telemetry.get_snapshot())

@app.route('/api/control', methods=['POST'])
def control():
    command = request.json
    if command['action'] == 'manual_pan':
        motor.set_velocity(command['velocity'])
    # ... handle other commands

# SocketIO events
@socketio.on('toggle_mode')
def handle_toggle():
    state_machine.toggle_manual_auto()
```

## Configuration Management

**config.yaml** - System-wide settings
```yaml
hardware:
  simulation_mode: false

pan_motor:
  encoder_counts_per_360: -8556
  gear_ratio: 135
  max_velocity_dps: 180
  max_acceleration_dps2: 360

tilt_servo:
  closed_loop: false  # Toggle ADS1115 feedback
  min_angle: 0
  max_angle: 180

cameras:
  hq:
    index: 0
    resolution: [1280, 720]
    framerate: 30
    fov_h: 60.0
    fov_v: 40.0
  gs:
    index: 1
    resolution: [640, 480]
    framerate: 60
    fov_h: 45.0
    fov_v: 34.0

acoustic:
  enabled: true
  odas_port: 9000
  mic_radius_mm: 46.5
  energy_threshold: 0.6
  frequency_range: [100, 500]

yolo:
  model_path: models/yolov11n-UAV-finetune_ncnn_model
  detection_confidence: 0.4
  tracking_confidence: 0.3

tracking:
  pid:
    kp: 1.2
    ki: 0.15
    kd: 0.05
  deadzone_pixels: 20
  max_velocity_dps: 180

web:
  host: 0.0.0.0
  port: 5000
  video_quality: 85

logging:
  level: INFO
  telemetry_rate_hz: 10
  csv_output: logs/telemetry.csv
```

## Performance Targets

- Main control loop: 50 Hz (20ms period)
- Detection pipeline: 30-60 FPS (camera dependent)
- Acoustic detection latency: < 100ms
- Pan slew to acoustic bearing: < 2 seconds for 180°
- Tracking accuracy: < 3 feet deviation @ 100 meters
- Web interface latency: < 100ms

## Error Handling

**Hardware Faults:**
- Camera disconnection → Switch to available camera
- Encoder read failure → Stop motor, log error
- Motor driver fault → Safe stop, enter ERROR state
- ODAS crash → Restart service, continue visual-only

**Software Faults:**
- YOLO inference timeout → Skip frame
- Config load failure → Use defaults
- Web server crash → Restart, maintain control

**Recovery Strategy:**
- Graceful degradation (continue with available sensors)
- Automatic service restart for non-critical failures
- Manual intervention required for ERROR state
- All faults logged with timestamp

## Testing Strategy

**Unit Tests:**
- Hardware abstraction layer interfaces
- Coordinate transformations
- PID controller logic
- State machine transitions

**Integration Tests:**
- Simulated full system operation
- Hardware-in-the-loop testing
- Performance benchmarking

**Field Tests:**
- Acoustic detection accuracy
- Visual tracking precision
- Multi-modal handoff timing
- Long-duration stability

## Future Enhancements

- Kalman filter for position/velocity estimation
- Stereo ranging using dual cameras
- Multiple target tracking
- Autonomous search patterns
- Machine learning for acoustic signature classification
- GPS integration for absolute positioning
- IMU for gimbal stabilization compensation
