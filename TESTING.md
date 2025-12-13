# PTDTS Testing & Validation

This document describes the testing and validation procedures for the PTDTS system.

## Test Suite Overview

The PTDTS project includes comprehensive testing utilities to validate system functionality before deployment.

### Automated Test Suite

**Location:** `tests/test_system_validation.py`

**Tests Included:**
1. **System Initialization** - Validates all hardware components initialize correctly
2. **Control Loop Timing** - Verifies 50Hz control loop performance
3. **Pan Controller Response** - Tests position and velocity control modes
4. **Tilt Controller Response** - Validates servo control
5. **State Machine Transitions** - Tests state changes and mode switching
6. **State Persistence** - Validates save/restore functionality
7. **Camera Frame Capture** - Tests both HQ and GS camera operation

### Running Tests

**Complete Validation Suite:**
```bash
python3 tests/test_system_validation.py
```

This will:
- Run all tests in simulation mode (no hardware required)
- Generate detailed timing and performance metrics
- Create a JSON report at `tests/validation_report.json`
- Provide pass/fail status for each test
- List warnings for performance issues

**Expected Duration:** ~30 seconds

### Test Report

After running, check the detailed report:
```bash
cat tests/validation_report.json
```

The report includes:
- Summary statistics (passed/failed/duration)
- Detailed results for each test
- Performance metrics
- Warnings and recommendations

### Performance Targets

**Control Loop:**
- Target: 50 Hz (20ms period)
- Acceptable range: 47.5 - 52.5 Hz (±5%)
- Maximum jitter: < 5ms

**Pan Controller:**
- Position accuracy: ±2°
- Velocity control: ±10% of target
- Response time: < 10s for 180° rotation

**State Machine:**
- State transition latency: < 100ms
- Manual mode activation: < 500ms

### Manual Testing

**Simulation Mode:**
```bash
python3 main.py --simulation
```

Then:
1. Access web interface at http://localhost:5000
2. Test manual mode toggle
3. Verify pan/tilt controls
4. Check telemetry updates
5. Test gamepad (if connected)

**Calibration Tests:**
```bash
# Test motor calibration
python3 calibration/calibrate_motor.py

# Test camera FOV
python3 calibration/calibrate_camera_fov.py --camera hq

# Test complete calibration workflow
python3 calibration/calibrate_all.py
```

### Integration Testing

**With Physical Hardware:**
1. Run calibration utilities first
2. Start system in production mode
3. Verify acoustic detection (with sound source)
4. Test visual detection (with YOLO model)
5. Validate tracking performance
6. Test state transitions end-to-end

### Pre-Deployment Checklist

- [ ] All automated tests pass
- [ ] Control loop timing within spec
- [ ] Hardware calibration complete
- [ ] State persistence working
- [ ] Web interface accessible
- [ ] Gamepad controls functional
- [ ] All components initialize without errors
- [ ] No warning messages in logs
- [ ] Telemetry data streaming correctly
- [ ] Camera feeds operational

### Troubleshooting Test Failures

**Import Errors:**
- Ensure all dependencies installed: `pip install -r requirements.txt`
- Check Python version: Python 3.8+

**Timing Issues:**
- System may be under heavy load
- Close other applications
- Rerun tests

**Hardware Errors in Simulation:**
- Should not occur - report as bug
- Check config.yaml has `simulation_mode: true`

**Camera Errors:**
- Normal in simulation mode if YOLO model missing
- Tests will skip visual detection if model not found

### Continuous Integration

For automated testing in CI/CD:
```bash
# Run tests with exit code
python3 tests/test_system_validation.py
echo $?  # 0 = pass, 1 = fail
```

### Performance Benchmarking

Tests automatically collect performance metrics:
- Control loop frequency and jitter
- State machine transition times
- Camera frame capture rates
- Memory usage
- CPU temperature (if available)

Review `tests/validation_report.json` for detailed benchmark data.

## Future Test Additions

Planned enhancements:
- Unit tests for individual components
- Mock hardware tests
- Network latency tests
- Long-duration stability tests
- Multi-threading stress tests
- Error recovery tests
