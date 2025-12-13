// PTDTS Web Interface - Main JavaScript
// Real-time telemetry and control

let socket;
let manualModeActive = false;
let startTime = Date.now();

// Initialize on page load
document.addEventListener('DOMContentLoaded', function() {
    initializeSocketIO();
    loadSystemConfig();
    startUptimeCounter();
    logEvent('Web interface loaded');
});

// SocketIO Connection
function initializeSocketIO() {
    socket = io();

    socket.on('connect', function() {
        updateConnectionStatus(true);
        logEvent('Connected to server');
    });

    socket.on('disconnect', function() {
        updateConnectionStatus(false);
        logEvent('Disconnected from server', 'warning');
    });

    socket.on('connected', function(data) {
        logEvent('SocketIO handshake complete');
    });

    socket.on('telemetry', function(data) {
        updateTelemetry(data);
    });

    socket.on('error', function(data) {
        logEvent('Error: ' + data.message, 'error');
    });

    // Request telemetry updates
    setInterval(function() {
        if (socket.connected) {
            socket.emit('request_telemetry', {});
        }
    }, 100); // 10 Hz updates
}

// Update connection status indicator
function updateConnectionStatus(connected) {
    const indicator = document.getElementById('status-indicator');
    const text = document.getElementById('status-text');

    if (connected) {
        indicator.classList.add('connected');
        indicator.classList.remove('disconnected');
        text.textContent = 'Connected';
    } else {
        indicator.classList.remove('connected');
        indicator.classList.add('disconnected');
        text.textContent = 'Disconnected';
    }
}

// Load system configuration
function loadSystemConfig() {
    fetch('/api/config')
        .then(response => response.json())
        .then(data => {
            if (data.status === 'ok') {
                updateSystemInfo(data.config);
            }
        })
        .catch(error => {
            logEvent('Failed to load config: ' + error, 'error');
        });
}

// Update system information display
function updateSystemInfo(config) {
    document.getElementById('sim-mode').textContent = config.simulation_mode ? 'Enabled' : 'Disabled';
    document.getElementById('max-pan-vel').textContent = config.pan_limits.max_velocity.toFixed(1) + ' deg/s';
    document.getElementById('max-pan-accel').textContent = config.pan_limits.max_acceleration.toFixed(1) + ' deg/s^2';
    document.getElementById('tilt-range').textContent =
        config.tilt_limits.min_angle + '-' + config.tilt_limits.max_angle + ' deg';

    const hqRes = config.cameras.hq;
    document.getElementById('hq-camera-res').textContent =
        hqRes.resolution[0] + 'x' + hqRes.resolution[1] + ' @ ' + hqRes.framerate + ' FPS';

    const gsRes = config.cameras.gs;
    document.getElementById('gs-camera-res').textContent =
        gsRes.resolution[0] + 'x' + gsRes.resolution[1] + ' @ ' + gsRes.framerate + ' FPS';
}

// Update telemetry displays
function updateTelemetry(data) {
    if (!data || data.error) {
        return;
    }

    // State information
    if (data.state) {
        updateStateDisplay(data.state);
    }

    // Pan axis
    if (data.pan) {
        updatePanDisplay(data.pan);
    }

    // Tilt axis
    if (data.tilt) {
        updateTiltDisplay(data.tilt);
    }

    // Detector
    if (data.detector) {
        updateDetectorDisplay(data.detector);
    }

    // Camera
    if (data.camera) {
        document.getElementById('active-camera').textContent =
            data.camera.active.toUpperCase() + ' Camera';
        document.getElementById('active-camera-name').textContent =
            data.camera.active.toUpperCase();
    }
}

// Update state machine display
function updateStateDisplay(state) {
    const stateName = state.current.toUpperCase();
    const duration = state.duration.toFixed(1);

    // Update state badge
    const stateBadge = document.getElementById('current-state');
    stateBadge.textContent = stateName;
    stateBadge.className = 'state-badge state-' + state.current.toLowerCase();

    // Update state info
    document.getElementById('state-value').textContent = stateName;
    document.getElementById('state-duration').textContent = duration + 's';
    document.getElementById('current-state-display').textContent = stateName;

    // Update target info
    if (state.target) {
        const targetText = `Class: ${state.target.class_name}, ` +
                          `Pos: (${state.target.center_x.toFixed(0)}, ${state.target.center_y.toFixed(0)}), ` +
                          `Conf: ${state.target.confidence.toFixed(2)}`;
        document.getElementById('target-info').textContent = targetText;
    } else {
        document.getElementById('target-info').textContent = 'No target';
    }

    // Update acoustic info
    if (state.pending_acoustic) {
        const acousticText = `Az: ${state.pending_acoustic.azimuth.toFixed(1)}°, ` +
                            `El: ${state.pending_acoustic.elevation.toFixed(1)}°, ` +
                            `Energy: ${state.pending_acoustic.energy.toFixed(2)}`;
        document.getElementById('acoustic-info').textContent = acousticText;
    } else {
        document.getElementById('acoustic-info').textContent = 'None';
    }

    // Update detection mode
    let detectionMode = 'Idle';
    if (stateName === 'DETECTING') {
        detectionMode = 'Visual Search';
    } else if (stateName === 'TRACKING') {
        detectionMode = 'Active Tracking';
    } else if (stateName === 'LISTENING') {
        detectionMode = 'Acoustic Monitoring';
    }
    document.getElementById('detection-mode').textContent = detectionMode;
}

// Update pan axis display
function updatePanDisplay(pan) {
    // Position and velocity
    document.getElementById('pan-position').textContent = pan.position.toFixed(2) + ' deg';
    document.getElementById('pan-velocity').textContent = pan.velocity.toFixed(2) + ' deg/s';

    // Mode
    const modeBadge = document.getElementById('pan-mode');
    modeBadge.textContent = pan.mode.toUpperCase();

    // Targets
    if (pan.target_position !== null && pan.target_position !== undefined) {
        document.getElementById('pan-target-pos').textContent = pan.target_position.toFixed(2) + ' deg';
    } else {
        document.getElementById('pan-target-pos').textContent = 'None';
    }

    if (pan.target_velocity !== null && pan.target_velocity !== undefined) {
        document.getElementById('pan-target-vel').textContent = pan.target_velocity.toFixed(2) + ' deg/s';
    } else {
        document.getElementById('pan-target-vel').textContent = 'None';
    }

    // Progress bar (0-360 degrees)
    const positionNormalized = ((pan.position % 360) + 360) % 360; // Normalize to 0-360
    const progressPercent = (positionNormalized / 360) * 100;
    document.getElementById('pan-progress').style.width = progressPercent + '%';

    // Encoder and PWM (if available)
    if (pan.encoder !== undefined) {
        document.getElementById('pan-encoder').textContent = pan.encoder;
    }
    if (pan.pwm !== undefined) {
        document.getElementById('pan-pwm').textContent = pan.pwm.toFixed(3);
    }
}

// Update tilt axis display
function updateTiltDisplay(tilt) {
    document.getElementById('tilt-angle').textContent = tilt.angle.toFixed(1) + ' deg';
    document.getElementById('tilt-target').textContent = tilt.target_angle.toFixed(1) + ' deg';

    // Progress bar (0-180 degrees)
    const progressPercent = (tilt.angle / 180) * 100;
    document.getElementById('tilt-progress').style.width = progressPercent + '%';

    // PWM (if available)
    if (tilt.pwm !== undefined) {
        document.getElementById('tilt-pwm').textContent = tilt.pwm + ' us';
    }
}

// Update detector display
function updateDetectorDisplay(detector) {
    const fps = detector.fps || 0.0;
    const inferenceTime = detector.inference_time || 0.0;

    document.getElementById('detector-fps').textContent = 'FPS: ' + fps.toFixed(1);
    document.getElementById('inference-time').textContent = 'Inference: ' + inferenceTime.toFixed(1) + 'ms';
    document.getElementById('detector-fps-value').textContent = fps.toFixed(1);
    document.getElementById('inference-time-value').textContent = inferenceTime.toFixed(1) + ' ms';
}

// Manual Mode Toggle
function toggleManualMode() {
    manualModeActive = !manualModeActive;

    fetch('/api/control/manual', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ enable: manualModeActive })
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'ok') {
            updateManualModeUI(manualModeActive);
            logEvent('Manual mode ' + (manualModeActive ? 'enabled' : 'disabled'));
        }
    })
    .catch(error => {
        logEvent('Failed to toggle manual mode: ' + error, 'error');
        manualModeActive = !manualModeActive; // Revert
    });
}

// Update manual mode UI
function updateManualModeUI(enabled) {
    const toggleBtn = document.getElementById('toggle-manual');
    const panSlider = document.getElementById('pan-velocity-ctrl');
    const tiltSlider = document.getElementById('tilt-angle-ctrl');
    const controlButtons = document.querySelectorAll('.btn-control');

    // Verify elements exist
    if (!panSlider) {
        logEvent('ERROR: Pan slider element not found!', 'error');
        console.error('Pan slider element (id=pan-velocity) not found in DOM');
        return;
    }
    if (!tiltSlider) {
        logEvent('ERROR: Tilt slider element not found!', 'error');
        console.error('Tilt slider element (id=tilt-angle-ctrl) not found in DOM');
        return;
    }

    if (enabled) {
        toggleBtn.textContent = 'Disable Manual';
        toggleBtn.style.backgroundColor = '#9d6b6b';

        // Enable sliders with explicit logging
        panSlider.disabled = false;
        panSlider.removeAttribute('disabled'); // Ensure attribute is removed
        tiltSlider.disabled = false;
        tiltSlider.removeAttribute('disabled');
        controlButtons.forEach(btn => btn.disabled = false);

        // Log slider states for debugging
        console.log('Manual mode ENABLED - Slider states:');
        console.log('  Pan slider disabled:', panSlider.disabled);
        console.log('  Tilt slider disabled:', tiltSlider.disabled);
        logEvent('Manual controls enabled (pan + tilt sliders active)', 'success');
    } else {
        toggleBtn.textContent = 'Enable Manual';
        toggleBtn.style.backgroundColor = '';

        // Disable sliders
        panSlider.disabled = true;
        panSlider.setAttribute('disabled', 'disabled'); // Ensure attribute is set
        tiltSlider.disabled = true;
        tiltSlider.setAttribute('disabled', 'disabled');
        controlButtons.forEach(btn => btn.disabled = true);

        // Reset sliders
        panSlider.value = 0;
        tiltSlider.value = 90;
        updatePanVelocity(0);
        updateTiltAngle(90);

        console.log('Manual mode DISABLED - Slider states:');
        console.log('  Pan slider disabled:', panSlider.disabled);
        console.log('  Tilt slider disabled:', tiltSlider.disabled);
    }
}

// Pan velocity control
function updatePanVelocity(value) {
    document.getElementById('pan-vel-display').textContent = value;

    // Send command immediately while dragging (only in manual mode)
    if (manualModeActive) {
        setPanVelocity(parseFloat(value));
    }
}

function setPanVelocity(velocity) {
    if (!manualModeActive) return;

    fetch('/api/control/pan', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ velocity: velocity })
    })
    .then(response => response.json())
    .then(data => {
        if (data.status !== 'ok') {
            logEvent('Pan control failed: ' + (data.message || 'Unknown error'), 'error');
        }
    })
    .catch(error => {
        logEvent('Pan control failed: ' + error, 'error');
    });
}

// Tilt angle control
function updateTiltAngle(value) {
    document.getElementById('tilt-angle-display').textContent = value;
}

function setTiltAngle(angle) {
    if (!manualModeActive) return;

    fetch('/api/control/tilt', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ angle: angle })
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'ok') {
            document.getElementById('tilt-angle-ctrl').value = angle;
            updateTiltAngle(angle);
        }
    })
    .catch(error => {
        logEvent('Tilt control failed: ' + error, 'error');
    });
}

// Handle slider input for pan with snap-back-to-zero
document.addEventListener('DOMContentLoaded', function() {
    const panSlider = document.getElementById('pan-velocity-ctrl');
    if (panSlider) {
        // Remove old change listener (handled by oninput in HTML now)

        // Add mouseup and touchend listeners to snap back to zero
        panSlider.addEventListener('mouseup', function() {
            // Snap back to zero and stop motor
            this.value = 0;
            updatePanVelocity(0);
        });

        panSlider.addEventListener('touchend', function() {
            // Snap back to zero and stop motor (for touch devices)
            this.value = 0;
            updatePanVelocity(0);
        });
    }

    const tiltSlider = document.getElementById('tilt-angle-ctrl');
    if (tiltSlider) {
        tiltSlider.addEventListener('change', function() {
            setTiltAngle(parseFloat(this.value));
        });
    }
});

// Event logging
function logEvent(message, level = 'info') {
    const logContainer = document.getElementById('event-log');
    const timestamp = new Date().toLocaleTimeString();

    const entry = document.createElement('div');
    entry.className = 'log-entry';

    const timeSpan = document.createElement('span');
    timeSpan.className = 'log-time';
    timeSpan.textContent = '[' + timestamp + ']';

    const messageSpan = document.createElement('span');
    messageSpan.className = 'log-message';
    if (level === 'error') {
        messageSpan.classList.add('text-error');
    } else if (level === 'warning') {
        messageSpan.classList.add('text-warning');
    } else if (level === 'success') {
        messageSpan.classList.add('text-success');
    }
    messageSpan.textContent = message;

    entry.appendChild(timeSpan);
    entry.appendChild(document.createTextNode(' '));
    entry.appendChild(messageSpan);

    // Remove "initialized" message if exists
    const firstEntry = logContainer.firstElementChild;
    if (firstEntry && firstEntry.textContent.includes('initialized')) {
        logContainer.removeChild(firstEntry);
    }

    logContainer.insertBefore(entry, logContainer.firstChild);

    // Limit log entries
    while (logContainer.children.length > 100) {
        logContainer.removeChild(logContainer.lastChild);
    }
}

// Clear event log
function clearEventLog() {
    const logContainer = document.getElementById('event-log');
    logContainer.innerHTML = '<div class="log-entry"><span class="log-time">[' +
        new Date().toLocaleTimeString() + ']</span> <span class="log-message">Log cleared</span></div>';
}

// Uptime counter
function startUptimeCounter() {
    setInterval(function() {
        const uptime = Date.now() - startTime;
        const seconds = Math.floor(uptime / 1000) % 60;
        const minutes = Math.floor(uptime / 60000) % 60;
        const hours = Math.floor(uptime / 3600000);

        const uptimeStr =
            String(hours).padStart(2, '0') + ':' +
            String(minutes).padStart(2, '0') + ':' +
            String(seconds).padStart(2, '0');

        document.getElementById('uptime').textContent = uptimeStr;
    }, 1000);
}

// Keyboard shortcuts
document.addEventListener('keydown', function(event) {
    if (!manualModeActive) return;

    switch(event.key) {
        case 'ArrowLeft':
            setPanVelocity(-30);
            break;
        case 'ArrowRight':
            setPanVelocity(30);
            break;
        case 'ArrowUp':
            setTiltAngle(0);
            break;
        case 'ArrowDown':
            setTiltAngle(180);
            break;
        case ' ':
            setPanVelocity(0);
            event.preventDefault();
            break;
    }
});

// State switching function
function switchState(stateName) {
    if (!stateName) return; // Empty selection

    fetch('/api/state/set', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ state: stateName })
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'ok') {
            logEvent('State switched to: ' + stateName.toUpperCase(), 'info');
            // Reset selector to default
            document.getElementById('state-selector').value = '';
        } else {
            logEvent('Failed to switch state: ' + data.message, 'error');
        }
    })
    .catch(error => {
        logEvent('Error switching state: ' + error, 'error');
    });
}

// Camera switching function
function switchCamera(cameraName) {
    fetch('/api/camera/switch', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ camera: cameraName })
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'ok') {
            logEvent('Switched to ' + cameraName.toUpperCase() + ' camera (manual override)', 'info');
            updateCameraStatus();
        } else {
            logEvent('Failed to switch camera: ' + data.message, 'error');
        }
    })
    .catch(error => {
        logEvent('Error switching camera: ' + error, 'error');
    });
}

// Camera auto mode function
function cameraAutoMode() {
    fetch('/api/camera/auto', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        }
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'ok') {
            logEvent('Camera manual override disabled (auto mode)', 'info');
            updateCameraStatus();
        } else {
            logEvent('Failed to enable camera auto mode: ' + data.message, 'error');
        }
    })
    .catch(error => {
        logEvent('Error enabling camera auto mode: ' + error, 'error');
    });
}

// Update camera status display
function updateCameraStatus() {
    fetch('/api/camera/status')
    .then(response => response.json())
    .then(data => {
        if (data.status === 'ok') {
            document.getElementById('active-camera-display').textContent = data.active_camera.toUpperCase();
            document.getElementById('camera-override-display').textContent = data.manual_override ? 'Yes' : 'No';

            // Highlight active camera button
            const hqBtn = document.getElementById('camera-hq');
            const gsBtn = document.getElementById('camera-gs');
            const autoBtn = document.getElementById('camera-auto');

            hqBtn.classList.remove('active');
            gsBtn.classList.remove('active');
            autoBtn.classList.remove('active');

            if (data.manual_override) {
                if (data.active_camera === 'hq') {
                    hqBtn.classList.add('active');
                } else if (data.active_camera === 'gs') {
                    gsBtn.classList.add('active');
                }
            } else {
                autoBtn.classList.add('active');
            }
        }
    })
    .catch(error => {
        console.error('Error updating camera status:', error);
    });
}

// Poll camera status periodically
setInterval(updateCameraStatus, 500);
