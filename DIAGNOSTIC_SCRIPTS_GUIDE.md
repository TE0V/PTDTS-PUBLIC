# PTDTS Diagnostic Scripts Guide

Quick reference for all diagnostic and test scripts in the repository.

## 🎯 SERVO (TILT) DIAGNOSTICS

### **test_servo_basic.py** ⭐ **RUN THIS FIRST FOR SERVO ISSUE**
**Status:** NEW - Ultra-simple servo diagnostic
**Purpose:** Minimal test to verify GPIO 18 PWM and servo wiring
**Tests:**
- GPIO 18 hardware PWM signal generation
- Servo receives and responds to signal (beep)
- Two-part test: static position + movement sequence

**When to run:** Servo not responding at all (no beep, no movement)
**What it does:**
1. Sends static center pulse (1500µs) - some servos beep immediately
2. If no beep, sweeps through full range - some servos only beep on movement
3. Interactive wiring verification prompts

**Expected output:**
- Servo beeps during TEST 1 or TEST 2 → GPIO 18 working, proceed to test_tilt_servo.py
- No beep after both tests → Wiring or power issue

```bash
sudo python3 test_servo_basic.py
```

---

### **test_tilt_servo.py**
**Status:** Existing - Use AFTER PWM working
**Purpose:** Full servo functionality test with analog feedback
**Tests:**
- Servo movement through full range
- S-curve motion profiles
- Analog position feedback (if ADS1115 connected)
- Commanded vs actual position accuracy

**When to run:** After confirming PWM works, to test servo control logic
**Modes:**
1. Automated test sequence (moves through predefined positions)
2. Interactive manual control (keyboard control)

```bash
sudo python3 test_tilt_servo.py
```

---

### **calibration/calibrate_servo.py**
**Status:** Optional - Only if using closed-loop feedback
**Purpose:** Calibrates ADS1115 analog feedback for closed-loop servo control
**When to run:** Only if `config.yaml → tilt_servo.closed_loop: true`
**What it does:**
- Moves servo 0° → 180° in steps
- Records voltage at each position
- Calculates voltage_min and voltage_max
- Updates config.yaml

**Current relevance:** LOW (closed_loop is disabled in config)

```bash
python3 calibration/calibrate_servo.py
```

---

## 🔄 PAN MOTOR (ENCODER) DIAGNOSTICS

### **test_encoder_direction.py** ⭐ **CRITICAL FOR PAN OSCILLATION**
**Status:** NEW - Part of oscillation diagnostic suite
**Purpose:** Verifies encoder direction matches motor direction
**Why critical:** Inverted encoder causes PID to fight itself → oscillation

**Tests:**
- CW rotation → counts should INCREASE (encoder.py negates to positive angle)
- CCW rotation → counts should DECREASE (encoder.py negates to negative angle)
- Provides clear pass/fail with fix instructions

**When to run:** Experiencing pan motor oscillation
**What to check:** Manual rotation test + optional motor-driven test

```bash
sudo python3 test_encoder_direction.py
```

---

### **test_backlash.py** ⭐ **CRITICAL FOR PAN OSCILLATION**
**Status:** NEW - Part of oscillation diagnostic suite
**Purpose:** Measures mechanical backlash in 135:1 gearbox
**Why critical:** Backlash causes oscillation during direction reversals

**Tests:**
- Moves to target angle
- Returns to start position
- Measures positional error (backlash)
- Tests multiple angles and directions

**When to run:** Oscillation persists after other fixes
**Interpretation:**
- <1° backlash: GOOD (unlikely to cause oscillation)
- 1-3° backlash: MODERATE (may contribute to oscillation)
- >3° backlash: LARGE (major contributor - hardware issue)

```bash
sudo python3 test_backlash.py
```

---

### **test_encoder_direct.py**
**Status:** Existing - Low-level encoder test
**Purpose:** Minimal dependency test of LS7366R SPI communication
**When to run:**
- Encoder not working at all
- Verify SPI communication
- Test if encoder counts changing

**Current relevance:** LOW (encoder is working in your system)

```bash
sudo python3 test_encoder_direct.py
```

---

### **test_encoder_modes.py**
**Status:** Existing - Historical troubleshooting
**Purpose:** Tests different LS7366R quadrature modes
**History:** Used to discover that only non-quadrature mode works reliably

**Current relevance:** OBSOLETE
**Reason:** Already determined non-quadrature mode (0x00) is only working mode
**Can delete:** Yes (unless you want to re-verify)

```bash
# OBSOLETE - non-quadrature mode already configured
sudo python3 test_encoder_modes.py
```

---

### **test_pan_motor.py**
**Status:** Existing - Manual motor control
**Purpose:** Bypasses PID/controller - direct PWM control
**Tests:**
- Motor direction (CW/CCW)
- PWM power levels
- Encoder reading accuracy
- Basic operation verification

**When to run:**
- Motor not responding to commands
- Verify motor wiring
- Test encoder accuracy during motion

**Controls:**
- w/s: Increase/decrease PWM
- a/d: Rotate left/right
- space: Stop
- r: Reset encoder

```bash
sudo python3 test_pan_motor.py
```

---

### **test_pan_tracking.py**
**Status:** Existing - Comprehensive integration test
**Purpose:** Full system test of pan motor + controller + tracking logic
**Tests:**
- Position control accuracy
- Velocity control
- Tracking behavior
- PID tuning verification

**When to run:** After basic motor/encoder work, to test full integration
**Current relevance:** MEDIUM (useful after oscillation fixed)

```bash
sudo python3 test_pan_tracking.py
```

---

## 📷 CAMERA CALIBRATION

### **calibrate_gs_camera.py** ⭐ **IMPORTANT FOR CAMERA SWITCHING**
**Status:** NEW - Part of oscillation diagnostic suite
**Purpose:** Calibrates optical offset between HQ and GS cameras
**Why important:** Prevents oscillation when switching cameras during tracking

**What it does:**
- Detects target on HQ camera
- Switches to GS camera
- Detects same target on GS camera
- Calculates pan/tilt offset
- Updates config.yaml

**When to run:** Before enabling camera auto-switching
**Current value:** `camera_offsets.gs: [0.0, 0.0]` (NOT CALIBRATED)

```bash
sudo python3 calibrate_gs_camera.py
```

---

### **calibration/calibrate_camera_fov.py**
**Status:** Existing - Camera FOV measurement
**Purpose:** Measures actual camera field of view angles
**When to run:**
- Camera FOV unknown
- Pixel-to-angle conversion inaccurate
- Tracking consistently off-target

**Current relevance:** MEDIUM (may need if tracking pointing is wrong)

```bash
python3 calibration/calibrate_camera_fov.py
```

---

### **calibration/calibrate_camera_fov_web.py**
**Status:** Existing - Web-based FOV calibration
**Purpose:** Same as calibrate_camera_fov.py but with web interface
**Advantage:** Visual feedback, easier to use

```bash
python3 calibration/calibrate_camera_fov_web.py
# Then open browser to http://pi-ip:5000
```

---

## 🎮 OTHER DIAGNOSTICS

### **test_gamepad.py**
**Status:** Existing - Manual control test
**Purpose:** Test Xbox controller input
**When to run:** Manual control not working
**Current relevance:** LOW (not related to current issues)

```bash
sudo python3 test_gamepad.py
```

---

### **test_state_persistence.py**
**Status:** Existing - State save/load test
**Purpose:** Verify runtime state is saved and restored correctly
**When to run:** State not persisting across reboots
**Current relevance:** LOW (not related to current issues)

```bash
python3 test_state_persistence.py
```

---

### **test_audio_array.py**
**Status:** Existing - Acoustic array test
**Purpose:** Test ReSpeaker microphone array
**Current relevance:** LOW (not related to servo/motor issues)

```bash
sudo python3 test_audio_array.py
```

---

### **calibration/calibrate_motor.py**
**Status:** Existing - Motor PID tuning
**Purpose:** Auto-tune pan motor PID parameters
**When to run:** After major mechanical changes or if PID poorly tuned
**Current relevance:** MEDIUM (may help with oscillation if PID is bad)

```bash
python3 calibration/calibrate_motor.py
```

---

### **calibration/calibrate_acoustic.py**
**Status:** Existing - Acoustic array calibration
**Purpose:** Calibrate microphone array for direction finding
**Current relevance:** LOW (not related to current issues)

```bash
python3 calibration/calibrate_acoustic.py
```

---

### **calibration/calibrate_all.py**
**Status:** Existing - Run all calibrations
**Purpose:** Orchestrates all calibration scripts
**When to run:** Fresh system setup or major hardware changes

```bash
python3 calibration/calibrate_all.py
```

---

## 🚨 PRIORITY FOR CURRENT ISSUES

### **SERVO NOT WORKING:**
1. ⭐⭐⭐ **test_servo_basic.py** - CRITICAL - Run first (verify wiring & GPIO 18)
2. **test_tilt_servo.py** - After servo beeps in basic test
3. **calibration/calibrate_servo.py** - Only if using closed-loop

### **PAN MOTOR OSCILLATION:**
1. ⭐⭐⭐ **test_encoder_direction.py** - CRITICAL - Check encoder not inverted
2. ⭐⭐⭐ **calibrate_gs_camera.py** - CRITICAL - Prevent camera switching oscillation
3. ⭐⭐ **test_backlash.py** - IMPORTANT - Measure mechanical backlash
4. **test_pan_motor.py** - Verify basic motor operation
5. **calibration/calibrate_motor.py** - Re-tune PID if needed

### **TRACKING ACCURACY:**
1. **calibration/calibrate_camera_fov.py** - Verify pixel-to-angle conversion
2. **calibrate_gs_camera.py** - Calibrate camera optical offsets
3. **test_pan_tracking.py** - Integration test

---

## 📝 OBSOLETE SCRIPTS (Can be removed)

- **test_encoder_modes.py** - Already determined non-quadrature mode works
  - Keep only for historical reference or if re-verifying encoder modes

---

## 💡 QUICK TROUBLESHOOTING FLOWCHART

```
SERVO NOT WORKING?
└─> Run test_servo_basic.py (verify wiring on GPIO 18)
    ├─> Servo beeps? → Run test_tilt_servo.py (full functionality test)
    └─> Never beeps? → Check wiring (Pin 12) and power supply

PAN MOTOR OSCILLATING?
├─> Run test_encoder_direction.py (check encoder not inverted)
├─> Run calibrate_gs_camera.py (calibrate camera offsets)
├─> Run test_backlash.py (measure mechanical backlash)
└─> If still oscillating → Run calibration/calibrate_motor.py (retune PID)

TRACKING INACCURATE?
├─> Run calibration/calibrate_camera_fov.py (verify FOV)
├─> Run calibrate_gs_camera.py (calibrate camera offsets)
└─> Run test_pan_tracking.py (integration test)

MOTOR NOT RESPONDING?
└─> Run test_pan_motor.py (direct PWM control, bypass PID)
```

---

## 📊 INFORMATION YOU CAN GAIN

### From Diagnostic Scripts:
- **test_servo_basic.py**: Whether GPIO 18 hardware PWM works and servo responds
- **test_encoder_direction.py**: If encoder direction is inverted (causes PID reversal)
- **test_backlash.py**: Amount of mechanical play in gears (causes oscillation)
- **test_pan_motor.py**: Basic motor/encoder functionality without PID

### From Calibration Scripts:
- **calibrate_gs_camera.py**: Pan/tilt offset between HQ and GS cameras
- **calibrate_camera_fov.py**: Actual camera field of view angles
- **calibrate_servo.py**: Voltage range for servo position feedback
- **calibrate_motor.py**: Optimal PID parameters for your specific motor/load

---

## 🔧 RECOMMENDED WORKFLOW FOR YOUR CURRENT ISSUES

1. **First: Fix Servo**
   ```bash
   sudo python3 test_servo_basic.py
   # Verify wiring on Pin 12 (GPIO 18)
   # Listen for servo beep during tests
   # If no beep → Check wiring and power
   ```

2. **Second: Diagnose Pan Oscillation**
   ```bash
   sudo python3 test_encoder_direction.py  # Check encoder direction
   sudo python3 calibrate_gs_camera.py     # Calibrate camera offset
   sudo python3 test_backlash.py           # Measure backlash
   ```

3. **Third: Test Full System**
   ```bash
   sudo python3 test_pan_tracking.py       # Integration test
   ```

4. **Optional: Re-tune if needed**
   ```bash
   python3 calibration/calibrate_motor.py  # Auto-tune PID
   ```
