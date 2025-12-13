# PTDTS - Pan Tilt Drone Detection and Tracking System

Affordable (~$1000) multi-modal drone detection and tracking system using acoustic detection, dual cameras, YOLO AI, and autonomous pan control on Raspberry Pi 5.

## Project Status

**Core Systems: COMPLETE**
- Hardware abstraction layer with simulation mode
- Motor/servo control with PID
- Dual camera system (HQ + Global Shutter)
- YOLO11n visual detection
- ODAS acoustic detection
- Comprehensive state machine
- Data logging and telemetry
- Web interface with real-time telemetry
- Manual control via web interface
- Main application entry point

**Complete:**
- Xbox controller/gamepad support for manual control
- Comprehensive hardware calibration utilities

**Remaining: TODO**
- Map view with target tracking (hybrid static/live maps approach)
- Field testing and tuning

## System Overview

### Four-State Operation

```
1. LISTENING  → Acoustic monitoring with ReSpeaker 4-mic array
      ↓ (Drone signature detected at azimuth θ)

2. PANNING    → Fast slew to acoustic bearing with approach deceleration
      ↓ (Arrived at position)

3. DETECTING  → HQ camera visual confirmation with YOLO11n
      ↓ (3 consecutive frames with confidence >0.4)

4. TRACKING   → Continuous velocity tracking with GS camera
      ↓ (Target lost for >2 seconds)

   Return to LISTENING
```

### Hardware Components

**Compute:**
- Raspberry Pi 5 (16GB RAM)
- 512GB NVMe SSD
- Active cooling
- LRS-150 24V Power Supply

**Pan Axis:**
- Pololu 30:1 Metal Gearmotor 37Dx68L mm 12V with 64 CPR encoder
- External 4.5:1 gear reduction (24T to 108T)
- Total reduction: 135:1
- DRV8874 motor driver (12V rail)
- LS7366R encoder buffer (SPI)

**Tilt Axis:**
- Axon MAX MK2 servo (6V rail)
- Analog position feedback via ADS1115 ADC (I2C)
- PWM control signal

**Cameras:**
- CAM0: Raspberry Pi HQ Camera (1280x720 @ 30 FPS for detection)
- CAM1: Raspberry Pi Global Shutter Camera (640x480 @ 60 FPS for tracking)
- 50mm baseline (over-under configuration)

**Audio:**
- ReSpeaker XMOS XVF3800 (4-microphone circular array)
- 46.5mm radius
- USB interface
- ODAS for direction of arrival (DOA)

**Power Regulation:**
- 5V Rail: Pololu D36V50F5 (Raspberry Pi)
- 6V Rail: Pololu D36V50F6 (Servo)
- 12V Rail: Pololu D36V50F12 (Motor driver)

## Software Architecture

### Layer 1: Hardware Abstraction (HAL)

**All hardware has simulation mode for testing without Pi!**

```python
# Example: Create hardware in simulation mode
from src.hardware import HardwareFactory
from src.utils import load_config

config = load_config()
config.simulation_mode = True  # Enable simulation

encoder = HardwareFactory.create_encoder(config)
motor = HardwareFactory.create_motor(config, encoder)
cameras = HardwareFactory.create_cameras(config)
```

**Interfaces:**
- `EncoderInterface` - LS7366R (SPI) or Simulated
- `MotorInterface` - DRV8874 or Simulated (accurate Pololu model)
- `ServoInterface` - Axon (PWM/closed-loop) or Simulated
- `CameraInterface` - Picamera2 or Simulated
- `AudioInterface` - ODAS or Simulated

### Layer 2: Control Systems

**PanController:**
- Position mode: Acoustic panning with trapezoidal profile
- Velocity mode: Continuous tracking (pixel error → velocity)
- Manual mode: User control
- Fixed 50Hz control loop

**TiltController:**
- Simple position control
- Optional closed-loop with ADS1115 feedback

**PID Controller:**
- Anti-windup integral clamping
- Derivative filtering
- Deadzone support
- Output limiting

### Layer 3: Detection Systems

**VisualDetector:**
- Multi-backend support: NCNN (optimized) or PyTorch (.pt)
- NCNN: 2-3x faster on ARM64 (15-30 FPS on Pi 5)
- PyTorch: Slower but easier to export (5-10 FPS on Pi 5)
- Auto-detects format from file extension
- Configurable confidence thresholds
- Drawing utilities for overlays
- Performance tracking (FPS, inference time)

**AcousticDetector:**
- ODAS integration via socket (port 9000)
- DOA extraction (azimuth/elevation)
- Energy filtering
- Bandpass 100-500 Hz (drone frequencies)

### Layer 4: State Machine

Coordinates all subsystems:

```python
LISTENING:  Acoustic monitoring, HQ camera idle
PANNING:    Fast slew to bearing, decelerate on approach
DETECTING:  HQ camera visual search, 3-frame confirmation
TRACKING:   GS camera high-speed tracking, velocity control
MANUAL:     User control via web/gamepad
ERROR:      Fault recovery
```

## Key Features

### 1. Accurate Motor Simulation

Based on Pololu 37D datasheet specs:
- No-load speed: 330 RPM gearbox → 14.7 deg/sec final shaft
- Stall torque: 140 kg⋅mm gearbox → 630 kg⋅mm final shaft
- Back-EMF and resistance modeling
- Static vs kinetic friction
- Current draw calculation (0.2A no-load, 5.5A stall)

### 2. Fixed Critical Bugs

From old system:
- **Motor direction mapping corrected** (IN1/IN2 were swapped)
- **Hardware encoder counting** replaces unreliable GPIO interrupts
- **PID I and D terms enabled** (were disabled causing steady-state error)

### 3. Professional Architecture

- Hardware abstraction for testability
- Configuration via YAML files
- Comprehensive logging (console, file, telemetry, detections)
- Thread-safe implementations
- Modular design

## Directory Structure

```
PTDTS/
├── config/
│   └── config.yaml              # System configuration
├── logs/                        # Generated at runtime
│   ├── ptdts.log               # Main log
│   ├── telemetry.csv           # Real-time data
│   └── detections.csv          # Detection events
├── models/
│   ├── yolov11n-UAV-finetune_ncnn_model.param  # YOLO NCNN model (user-provided)
│   └── yolov11n-UAV-finetune_ncnn_model.bin    # YOLO NCNN model binary
├── src/
│   ├── hardware/
│   │   ├── interfaces.py       # Hardware interfaces
│   │   ├── encoder.py          # LS7366R + simulated
│   │   ├── motor.py            # DRV8874 + simulated
│   │   ├── motor_simulation.py # Accurate Pololu model
│   │   ├── servo.py            # Axon + simulated
│   │   ├── camera.py           # Picamera2 + simulated + manager
│   │   ├── acoustic.py         # ODAS + simulated
│   │   ├── ads1115.py          # ADC for servo feedback
│   │   └── factory.py          # Hardware factory
│   ├── control/
│   │   ├── pid.py              # PID controller
│   │   ├── pan_controller.py   # Pan axis control
│   │   └── tilt_controller.py  # Tilt axis control
│   ├── detection/
│   │   └── visual_detector.py  # YOLO integration
│   ├── state/
│   │   └── state_machine.py    # State machine
│   ├── web/
│   │   └── app.py              # Flask web application
│   └── utils/
│       ├── config_loader.py    # Configuration
│       └── logger.py           # Logging utilities
├── templates/
│   └── index.html               # Web interface HTML
├── static/
│   ├── css/
│   │   └── style.css           # Web interface styling
│   └── js/
│       └── main.js             # Web interface JavaScript
├── main.py                      # Main application entry point
├── ARCHITECTURE.md              # Detailed design document
├── README.md                    # This file
└── requirements.txt             # Python dependencies
```

## Installation

### 1. Prerequisites (Raspberry Pi 5)

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install system packages
sudo apt install -y \
    python3-numpy python3-scipy python3-pandas python3-matplotlib \
    python3-opencv python3-picamera2 \
    libopenblas-dev \
    git

# Note: libatlas-base-dev is deprecated on modern Raspberry Pi OS
# OpenBLAS provides better performance on ARM64 (Pi 5)

# Install ODAS dependencies
sudo apt install -y \
    cmake \
    build-essential \
    libasound2-dev \
    libpulse-dev \
    libfftw3-dev \
    libconfig-dev \
    libjson-c-dev
```

**Enable SPI and I2C (Required for LS7366R encoder and ADS1115 ADC):**

```bash
# Enable SPI and I2C interfaces
sudo raspi-config nonint do_spi 0
sudo raspi-config nonint do_i2c 0

# Verify SPI is enabled
ls /dev/spidev*
# Should show: /dev/spidev0.0 /dev/spidev0.1

# Verify I2C is enabled
ls /dev/i2c*
# Should show: /dev/i2c-1 (and possibly /dev/i2c-0)

# If the above commands don't work, enable manually:
# sudo nano /boot/firmware/config.txt
# Add or uncomment these lines:
#   dtparam=spi=on
#   dtparam=i2c_arm=on
# Then reboot: sudo reboot
```

**Install ODAS (Open embeddeD Audition System):**

ODAS provides sound source localization for the ReSpeaker 4-mic array.

```bash
# Clone ODAS repository
cd /opt
sudo git clone https://github.com/introlab/odas.git
cd odas

# Build ODAS
sudo mkdir build
cd build
sudo cmake ..
sudo make -j4

# Install (creates /usr/local/bin/odaslive)
sudo make install

# Verify installation
which odaslive
# Should output: /usr/local/bin/odaslive

# Create config directory
sudo mkdir -p /opt/odas/config
```

**Configure ODAS for ReSpeaker:**

PTDTS will auto-generate the ODAS configuration, but you can create a custom config:

```bash
sudo nano /opt/odas/config/ptdts_respeaker.cfg
# The system will generate this automatically if not present
```

**Test ReSpeaker detection:**

```bash
# List audio devices (find your ReSpeaker)
arecord -l

# ReSpeaker should appear as something like:
# card 2: ArrayUAC10 [ReSpeaker 4 Mic Array (UAC1.0)], device 0: USB Audio [USB Audio]
```

### 2. Clone Repository

```bash
git clone https://github.com/TE0V/PTDTS.git
cd PTDTS
```

### 3. Install Python Dependencies

**For Raspberry Pi (Recommended - No venv):**

Since this is a dedicated system using hardware interfaces and system packages,
install directly without a virtual environment:

```bash
# Install remaining Python packages
pip3 install -r requirements.txt --break-system-packages
```

The `--break-system-packages` flag is required on Raspberry Pi OS Bookworm and later.
This is safe for a dedicated system running only PTDTS.

**For Development/Testing (Optional venv):**

Only use a virtual environment if you're developing on a non-Pi system:

```bash
# Development only - not recommended for production Pi
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

**Note:** System packages (opencv, picamera2) installed via apt are automatically
available to pip without a venv. A venv would require `--system-site-packages`
which defeats its purpose.

### 4. Configure System

Edit `config/config.yaml` to match your setup:

```yaml
# Set simulation mode for testing without hardware
simulation_mode: false  # Set to true for simulation

# Adjust GPIO pins if different
pan_motor:
  gpio_in1: 27
  gpio_in2: 22
  # ... other settings

# Configure cameras
cameras:
  hq:
    index: 0
    resolution: [1280, 720]
    framerate: 30
  gs:
    index: 1
    resolution: [640, 480]
    framerate: 60
```

### 5. Add YOLO Model

Place your trained YOLO11n model in `models/`. Two formats are supported:

**Option A: NCNN Format (Recommended for Pi 5 - 2-3x faster)**
```bash
# Copy NCNN model files (you need both .param and .bin)
# Note: The model directory should contain the .param and .bin files
cp /path/to/your/yolov11n-UAV-finetune_ncnn_model.param models/
cp /path/to/your/yolov11n-UAV-finetune_ncnn_model.bin models/

# Optional: Copy class names file
cp /path/to/your/yolov11n-UAV-finetune_ncnn_model.txt models/

# The config.yaml already uses NCNN model by default:
# model_path: models/yolov11n-UAV-finetune_ncnn_model
```

**Option B: PyTorch Format (Slower but simpler)**
```bash
# Copy PyTorch model
cp /path/to/your/yolov11n-UAV-finetune.pt models/

# Update config.yaml to use PyTorch model:
# model_path: models/yolov11n-UAV-finetune.pt
```

**Converting PyTorch to NCNN:**

If you have a `.pt` file and want to convert it to NCNN format for better performance:

```bash
# Install ultralytics if not already installed
pip3 install ultralytics

# Export to NCNN format
yolo export model=models/yolov11n-UAV-finetune.pt format=ncnn

# This creates a directory with the model files:
# - yolov11n-UAV-finetune_ncnn_model/model.ncnn.param
# - yolov11n-UAV-finetune_ncnn_model/model.ncnn.bin

# Rename and move to models/
mv yolov11n-UAV-finetune_ncnn_model/model.ncnn.param models/yolov11n-UAV-finetune_ncnn_model.param
mv yolov11n-UAV-finetune_ncnn_model/model.ncnn.bin models/yolov11n-UAV-finetune_ncnn_model.bin
```

**Performance Comparison on Raspberry Pi 5:**
- NCNN: 15-30 FPS (recommended)
- PyTorch (.pt): 5-10 FPS

## Usage

### Simulation Mode (No Hardware Required)

Perfect for development and testing - runs complete system with simulated hardware:

```bash
# Run with simulation mode (no hardware required)
python3 main.py --simulation

# Or set in config.yaml: simulation_mode: true
python3 main.py
```

Access web interface at: http://localhost:5000

### Production Mode (With Hardware)

```bash
# Run with real hardware (default from config)
python3 main.py

# Specify custom config file
python3 main.py --config /path/to/config.yaml

# Override web interface settings
python3 main.py --host 0.0.0.0 --port 8080
```

### Web Interface Features

- Real-time video feed with detection overlays
- Live telemetry updates (10 Hz)
- Pan/tilt position and velocity monitoring
- State machine status and history
- Manual control mode with keyboard/mouse
- System performance metrics
- Event logging

Access the web interface at the configured host:port (default: http://localhost:5000)

### Gamepad/Xbox Controller Support

PTDTS supports Xbox-compatible game controllers for intuitive manual control:

**Control Mapping:**
- **Left Stick X-axis**: Pan velocity (left/right rotation)
- **Right Stick Y-axis**: Tilt position adjustment (up/down)
- **Left Trigger**: Fine control mode (reduces sensitivity to 30%)
- **Right Trigger**: Turbo mode (doubles sensitivity)
- **A Button**: Toggle between Manual and Auto modes
- **B Button**: Emergency stop (halt all motion)
- **X Button**: Center pan to 0°
- **Y Button**: Reset tilt to 90° (horizontal)
- **Start Button**: Quick center (pan 0°, tilt 90°)
- **Back Button**: Return to listening state
- **LB/RB Bumpers**: Step pan left/right by 15°
- **D-Pad**: Fine pan/tilt adjustments

**Setup:**
1. Connect Xbox controller via USB or Bluetooth
2. Ensure gamepad is enabled in `config/config.yaml`
3. Start PTDTS - gamepad will be auto-detected
4. Press **A button** to enter manual control mode
5. Use sticks to control pan/tilt
6. Press **A button** again to return to auto mode

**Configuration:**
All gamepad settings can be adjusted in `config/config.yaml` under the `gamepad:` section:
- Deadzone thresholds
- Sensitivity multipliers
- Button mappings
- Fine control/turbo settings

## Configuration

All parameters are in `config/config.yaml`:

- **Hardware settings:** GPIO pins, SPI/I2C addresses
- **Motor tuning:** PID gains, speed limits, friction compensation
- **Servo settings:** PWM range, closed-loop option
- **Camera settings:** Resolution, FPS, FOV
- **Detection thresholds:** YOLO confidence, acoustic energy
- **Tracking parameters:** Pixel-to-velocity gain, deadzone
- **State machine:** Timeouts, confirmation frames

## Calibration

### Motor Calibration

Measure encoder counts per 360° rotation:

```bash
python3 calibration/calibrate_motor.py
```

Expected value: ~-2139 counts (with 135:1 ratio and non-quadrature mode)

**Note:** The LS7366R is configured in non-quadrature mode (not x1/x2/x4 quadrature) because quadrature modes were causing the encoder to freeze. Non-quadrature mode provides excellent resolution (0.168° per count) and is the only mode that works reliably on this hardware. The encoder.py module negates raw counts to match the expected physical direction (CW rotation = positive angle increase).

### Camera FOV Calibration

Measure field of view for accurate angle calculations:

```bash
# TODO: Add camera calibration utility
python3 calibrate_cameras.py
```

## Performance Targets

- Main control loop: 50 Hz (20ms period)
- Detection pipeline: 30-60 FPS (camera dependent)
- Acoustic latency: < 100ms
- Pan slew to acoustic: < 2 seconds for 180°
- **Tracking accuracy: < 3 feet deviation @ 100 meters**

## Troubleshooting

### LS7366R Encoder Not Reading

**1. Check SPI is enabled:**
```bash
ls /dev/spidev*
# Should show /dev/spidev0.0 and /dev/spidev0.1
```

**2. If SPI devices don't exist, enable SPI:**
```bash
sudo raspi-config nonint do_spi 0
sudo reboot
```

**3. Verify SPI communication:**
```bash
# Install spidev Python package if not already installed
pip3 install spidev

# Test SPI loopback (connect MISO to MOSI for testing)
python3 -c "import spidev; spi = spidev.SpiDev(); spi.open(0, 0); print('SPI OK')"
```

**4. Check wiring:**
- Pin 19 (GPIO 10) → LS7366R MOSI
- Pin 21 (GPIO 9) → LS7366R MISO
- Pin 23 (GPIO 11) → LS7366R SCK
- Pin 24 (GPIO 8) → LS7366R CS
- Pin 4 (5V) → LS7366R VCC
- Pin 9 (GND) → LS7366R GND

### Cameras Not Detected

List available cameras:
```bash
libcamera-hello --list-cameras
```

### ODAS Not Starting

Check USB audio devices:
```bash
arecord -l
```

## Development

### Running Tests

```bash
# TODO: Add tests
pytest tests/
```

### Linting

```bash
flake8 src/
black src/
```

## Project Timeline

**Phase 1: Core Systems** (COMPLETE)
- Hardware abstraction
- Motor/servo control
- Cameras and detection
- State machine

**Phase 2: Integration** (COMPLETE)
- Web interface with real-time telemetry
- Manual control via web interface
- Main application entry point
- Video streaming with detection overlays

**Phase 3: Refinement** (IN PROGRESS)
- ✓ Gamepad/Xbox controller support
- Calibration tools (TODO)
- Field testing (TODO)
- PID tuning (ongoing)
- Performance optimization (ongoing)
- Multi-target tracking (future)

## Contributing

This is a personal project, but feedback and suggestions are welcome!

## License

[Add license information]

## Acknowledgments

- Pololu for excellent motor documentation
- Ultralytics for YOLO
- ODAS team for acoustic localization
- Raspberry Pi Foundation

## Contact

[Add contact information]
