#!/usr/bin/env python3
"""
Web-Based Camera FOV Calibration Tool for PTDTS
Measures field of view for both HQ and GS cameras via web interface

Usage:
    python3 calibration/calibrate_camera_fov_web.py [--camera hq|gs] [--port 8080]

Then open http://<pi-ip-address>:8080 in your browser

This tool will:
1. Stream live camera feed to your browser
2. Guide you to measure known-width object at known distance
3. Accept clicks in browser to mark measurement points
4. Calculate horizontal and vertical FOV
5. Display results for updating config.yaml
"""

import sys
import cv2
import numpy as np
import argparse
import math
import time
import threading
from pathlib import Path
from flask import Flask, render_template_string, Response, jsonify, request
from flask_cors import CORS

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.utils.config_loader import load_config
from src.utils.logger import setup_logging
from src.hardware.factory import HardwareFactory
import logging

logger = logging.getLogger(__name__)


# HTML template for web interface
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>PTDTS Camera FOV Calibration</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #1a1a1a;
            color: #e0e0e0;
        }
        .container {
            max-width: 1400px;
            margin: 0 auto;
        }
        h1 {
            color: #4CAF50;
            margin-bottom: 10px;
        }
        .status {
            background-color: #2a2a2a;
            padding: 15px;
            border-radius: 5px;
            margin-bottom: 20px;
        }
        .video-container {
            position: relative;
            display: inline-block;
            border: 2px solid #4CAF50;
            border-radius: 5px;
            background-color: #000;
        }
        #videoFeed {
            display: block;
            max-width: 100%;
            cursor: crosshair;
        }
        .overlay {
            position: absolute;
            top: 0;
            left: 0;
            pointer-events: none;
        }
        .controls {
            background-color: #2a2a2a;
            padding: 20px;
            border-radius: 5px;
            margin-top: 20px;
        }
        .form-group {
            margin-bottom: 15px;
        }
        label {
            display: block;
            margin-bottom: 5px;
            color: #aaa;
        }
        input {
            width: 300px;
            padding: 8px;
            background-color: #333;
            border: 1px solid #555;
            border-radius: 3px;
            color: #fff;
        }
        button {
            background-color: #4CAF50;
            color: white;
            padding: 10px 20px;
            border: none;
            border-radius: 3px;
            cursor: pointer;
            font-size: 16px;
            margin-right: 10px;
        }
        button:hover {
            background-color: #45a049;
        }
        button:disabled {
            background-color: #666;
            cursor: not-allowed;
        }
        .instruction {
            background-color: #2a4a2a;
            padding: 15px;
            border-left: 4px solid #4CAF50;
            margin-bottom: 20px;
            border-radius: 3px;
        }
        .marker {
            width: 10px;
            height: 10px;
            background-color: red;
            border: 2px solid white;
            border-radius: 50%;
            position: absolute;
            transform: translate(-50%, -50%);
            pointer-events: none;
        }
        .results {
            background-color: #2a2a2a;
            padding: 20px;
            border-radius: 5px;
            margin-top: 20px;
            display: none;
        }
        .results pre {
            background-color: #1a1a1a;
            padding: 10px;
            border-radius: 3px;
            overflow-x: auto;
        }
        .step {
            display: none;
        }
        .step.active {
            display: block;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🎥 PTDTS Camera FOV Calibration</h1>
        <div class="status">
            <strong>Camera:</strong> <span id="cameraName">{{ camera_name }}</span> |
            <strong>Resolution:</strong> <span id="resolution">{{ resolution }}</span> |
            <strong>Current FOV:</strong> <span id="currentFov">{{ current_fov }}</span>
        </div>

        <div class="video-container" id="videoContainer">
            <img id="videoFeed" src="/video_feed">
            <canvas id="overlay" class="overlay"></canvas>
        </div>

        <!-- Step 1: Setup -->
        <div id="step1" class="step active">
            <div class="controls">
                <div class="instruction">
                    <h2>📋 Setup Instructions</h2>
                    <p>You will need:</p>
                    <ul>
                        <li>A calibration object of known width (e.g., ruler, paper, poster)</li>
                        <li>Tape measure to measure distance from camera</li>
                    </ul>
                    <p><strong>Steps:</strong></p>
                    <ol>
                        <li>Place calibration object at known distance from camera</li>
                        <li>Align object horizontally for horizontal FOV measurement</li>
                        <li>Click object edges in the video feed</li>
                        <li>Repeat vertically for vertical FOV</li>
                    </ol>
                </div>
                <button onclick="startCalibration()">Start Calibration</button>
            </div>
        </div>

        <!-- Step 2: Horizontal FOV Setup -->
        <div id="step2" class="step">
            <div class="controls">
                <div class="instruction">
                    <h2>📏 Horizontal FOV Measurement</h2>
                    <p>Place your calibration object horizontally in view of the camera.</p>
                </div>
                <div class="form-group">
                    <label for="objectWidth">Calibration object width (mm):</label>
                    <input type="number" id="objectWidth" placeholder="e.g., 210 for A4 paper">
                </div>
                <div class="form-group">
                    <label for="distance">Distance from camera to object (mm):</label>
                    <input type="number" id="distance" placeholder="e.g., 1000 for 1 meter">
                </div>
                <button onclick="startHorizontalMeasurement()">Continue to Measurement</button>
            </div>
        </div>

        <!-- Step 3: Horizontal FOV Clicks -->
        <div id="step3" class="step">
            <div class="controls">
                <div class="instruction">
                    <h2>🖱️ Click Edges</h2>
                    <p id="clickInstruction">Click the LEFT edge of the calibration object in the video feed above.</p>
                    <p><strong>Clicks:</strong> <span id="clickCount">0</span> / 2</p>
                </div>
                <button onclick="resetClicks()">Reset Clicks</button>
            </div>
        </div>

        <!-- Step 4: Vertical FOV Setup -->
        <div id="step4" class="step">
            <div class="controls">
                <div class="instruction">
                    <h2>📐 Vertical FOV Measurement</h2>
                    <p>Rotate your calibration object 90° (vertical orientation) or use a different object.</p>
                </div>
                <div class="form-group">
                    <label for="objectHeight">Calibration object height (mm):</label>
                    <input type="number" id="objectHeight" placeholder="Leave blank to use same as width">
                </div>
                <button onclick="startVerticalMeasurement()">Continue to Measurement</button>
            </div>
        </div>

        <!-- Step 5: Vertical FOV Clicks -->
        <div id="step5" class="step">
            <div class="controls">
                <div class="instruction">
                    <h2>🖱️ Click Edges</h2>
                    <p id="clickInstructionV">Click the TOP edge of the calibration object in the video feed above.</p>
                    <p><strong>Clicks:</strong> <span id="clickCountV">0</span> / 2</p>
                </div>
                <button onclick="resetClicksV()">Reset Clicks</button>
            </div>
        </div>

        <!-- Results -->
        <div id="results" class="results">
            <h2>✅ Calibration Results</h2>
            <div id="resultsContent"></div>
            <button onclick="location.reload()">Calibrate Again</button>
        </div>
    </div>

    <script>
        let currentStep = 1;
        let horizontalPoints = [];
        let verticalPoints = [];
        let objectWidth = 0;
        let objectHeight = 0;
        let distance = 0;
        let imageWidth = 0;
        let imageHeight = 0;

        const videoFeed = document.getElementById('videoFeed');
        const overlay = document.getElementById('overlay');
        const ctx = overlay.getContext('2d');

        // Update canvas size to match video
        videoFeed.onload = function() {
            overlay.width = videoFeed.width;
            overlay.height = videoFeed.height;
            imageWidth = videoFeed.width;
            imageHeight = videoFeed.height;
        };

        // Handle video feed clicks
        videoFeed.addEventListener('click', function(e) {
            const rect = videoFeed.getBoundingClientRect();
            const scaleX = videoFeed.naturalWidth / rect.width;
            const scaleY = videoFeed.naturalHeight / rect.height;
            const x = (e.clientX - rect.left) * scaleX;
            const y = (e.clientY - rect.top) * scaleY;

            if (currentStep === 3) {
                handleHorizontalClick(x, y);
            } else if (currentStep === 5) {
                handleVerticalClick(x, y);
            }
        });

        function showStep(step) {
            document.querySelectorAll('.step').forEach(el => el.classList.remove('active'));
            document.getElementById('step' + step).classList.add('active');
            currentStep = step;
        }

        function startCalibration() {
            showStep(2);
        }

        function startHorizontalMeasurement() {
            objectWidth = parseFloat(document.getElementById('objectWidth').value);
            distance = parseFloat(document.getElementById('distance').value);

            if (!objectWidth || !distance) {
                alert('Please enter object width and distance');
                return;
            }

            horizontalPoints = [];
            showStep(3);
        }

        function handleHorizontalClick(x, y) {
            horizontalPoints.push({x: x, y: y});
            drawMarker(x, y);

            document.getElementById('clickCount').textContent = horizontalPoints.length;

            if (horizontalPoints.length === 1) {
                document.getElementById('clickInstruction').textContent =
                    'Click the RIGHT edge of the calibration object.';
            } else if (horizontalPoints.length === 2) {
                // Calculate horizontal FOV
                calculateHorizontalFOV();
                showStep(4);
            }
        }

        function resetClicks() {
            horizontalPoints = [];
            document.getElementById('clickCount').textContent = '0';
            document.getElementById('clickInstruction').textContent =
                'Click the LEFT edge of the calibration object in the video feed above.';
            clearCanvas();
        }

        function startVerticalMeasurement() {
            let heightInput = document.getElementById('objectHeight').value;
            objectHeight = heightInput ? parseFloat(heightInput) : objectWidth;

            verticalPoints = [];
            clearCanvas();
            showStep(5);
        }

        function handleVerticalClick(x, y) {
            verticalPoints.push({x: x, y: y});
            drawMarker(x, y);

            document.getElementById('clickCountV').textContent = verticalPoints.length;

            if (verticalPoints.length === 1) {
                document.getElementById('clickInstructionV').textContent =
                    'Click the BOTTOM edge of the calibration object.';
            } else if (verticalPoints.length === 2) {
                // Calculate vertical FOV
                calculateVerticalFOV();
            }
        }

        function resetClicksV() {
            verticalPoints = [];
            document.getElementById('clickCountV').textContent = '0';
            document.getElementById('clickInstructionV').textContent =
                'Click the TOP edge of the calibration object in the video feed above.';
            clearCanvas();
        }

        function drawMarker(x, y) {
            const rect = videoFeed.getBoundingClientRect();
            const scaleX = rect.width / videoFeed.naturalWidth;
            const scaleY = rect.height / videoFeed.naturalHeight;

            ctx.fillStyle = 'red';
            ctx.strokeStyle = 'white';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.arc(x * scaleX, y * scaleY, 5, 0, 2 * Math.PI);
            ctx.fill();
            ctx.stroke();
        }

        function clearCanvas() {
            ctx.clearRect(0, 0, overlay.width, overlay.height);
        }

        function calculateHorizontalFOV() {
            const pixelsH = Math.abs(horizontalPoints[1].x - horizontalPoints[0].x);
            const fovH = calculateFOV(objectWidth, distance, pixelsH, imageWidth);

            fetch('/api/set_horizontal_fov', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    fov: fovH,
                    pixels: pixelsH,
                    object_width: objectWidth,
                    distance: distance
                })
            });

            console.log('Horizontal FOV:', fovH.toFixed(2), '°');
        }

        function calculateVerticalFOV() {
            const pixelsV = Math.abs(verticalPoints[1].y - verticalPoints[0].y);
            const fovV = calculateFOV(objectHeight, distance, pixelsV, imageHeight);

            fetch('/api/set_vertical_fov', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    fov: fovV,
                    pixels: pixelsV,
                    object_height: objectHeight,
                    distance: distance
                })
            })
            .then(() => fetch('/api/get_results'))
            .then(response => response.json())
            .then(data => {
                displayResults(data);
            });
        }

        function calculateFOV(objectSize, dist, pixels, imageSize) {
            const sensorSize = (objectSize * imageSize) / pixels;
            const fovRadians = 2 * Math.atan(sensorSize / (2 * dist));
            return fovRadians * (180 / Math.PI);
        }

        function displayResults(data) {
            const content = `
                <p><strong>Camera:</strong> ${data.camera}</p>
                <p><strong>Resolution:</strong> ${data.resolution}</p>
                <p><strong>Measured FOV:</strong></p>
                <ul>
                    <li>Horizontal: ${data.fov_h.toFixed(2)}° (object spans ${data.pixels_h} pixels, ${data.percent_h.toFixed(1)}% of frame width)</li>
                    <li>Vertical: ${data.fov_v.toFixed(2)}° (object spans ${data.pixels_v} pixels, ${data.percent_v.toFixed(1)}% of frame height)</li>
                    <li>Aspect ratio: ${data.aspect_ratio.toFixed(3)}</li>
                </ul>
                <h3>Configuration Update</h3>
                <p>Update <code>config/config.yaml</code> in <code>cameras.${data.camera}</code> section:</p>
                <pre>fov_h: ${data.fov_h.toFixed(1)}
fov_v: ${data.fov_v.toFixed(1)}</pre>
            `;

            document.getElementById('resultsContent').innerHTML = content;
            document.getElementById('results').style.display = 'block';
            showStep(1); // Hide step display
        }
    </script>
</body>
</html>
"""


class WebCalibration:
    """Web-based camera FOV calibration"""

    def __init__(self, camera_name='hq', port=8080):
        """
        Initialize web calibration

        Args:
            camera_name: 'hq' or 'gs'
            port: Web server port
        """
        self.camera_name = camera_name
        self.port = port
        self.config = None
        self.camera_manager = None
        self.camera = None

        # Calibration data
        self.fov_h = None
        self.fov_v = None
        self.pixels_h = None
        self.pixels_v = None
        self.object_width = None
        self.object_height = None
        self.distance = None

        # Flask app
        self.app = Flask(__name__)
        CORS(self.app)
        self.setup_routes()

    def setup_routes(self):
        """Setup Flask routes"""

        @self.app.route('/')
        def index():
            """Main calibration page"""
            cam_config = self.config.cameras.hq if self.camera_name == 'hq' else self.config.cameras.gs
            return render_template_string(
                HTML_TEMPLATE,
                camera_name=self.camera_name.upper(),
                resolution=f"{cam_config.resolution[0]}x{cam_config.resolution[1]}",
                current_fov=f"{cam_config.fov_h}° H x {cam_config.fov_v}° V"
            )

        @self.app.route('/video_feed')
        def video_feed():
            """Video streaming route"""
            return Response(
                self.generate_frames(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )

        @self.app.route('/api/set_horizontal_fov', methods=['POST'])
        def set_horizontal_fov():
            """Receive horizontal FOV measurement"""
            data = request.json
            self.fov_h = data['fov']
            self.pixels_h = data['pixels']
            self.object_width = data['object_width']
            self.distance = data['distance']
            logger.info(f"Horizontal FOV: {self.fov_h:.2f}° ({self.pixels_h} pixels)")
            return jsonify({'status': 'ok'})

        @self.app.route('/api/set_vertical_fov', methods=['POST'])
        def set_vertical_fov():
            """Receive vertical FOV measurement"""
            data = request.json
            self.fov_v = data['fov']
            self.pixels_v = data['pixels']
            self.object_height = data['object_height']
            logger.info(f"Vertical FOV: {self.fov_v:.2f}° ({self.pixels_v} pixels)")
            return jsonify({'status': 'ok'})

        @self.app.route('/api/get_results')
        def get_results():
            """Get calibration results"""
            cam_config = self.config.cameras.hq if self.camera_name == 'hq' else self.config.cameras.gs
            width, height = cam_config.resolution

            return jsonify({
                'camera': self.camera_name,
                'resolution': f"{width}x{height}",
                'fov_h': self.fov_h,
                'fov_v': self.fov_v,
                'pixels_h': self.pixels_h,
                'pixels_v': self.pixels_v,
                'percent_h': (self.pixels_h / width) * 100,
                'percent_v': (self.pixels_v / height) * 100,
                'aspect_ratio': self.fov_h / self.fov_v if self.fov_v else 0
            })

    def generate_frames(self):
        """
        Generate frames for video streaming

        Yields:
            JPEG frames in multipart format
        """
        while True:
            try:
                # Capture frame
                frame = self.camera.capture_array()

                if frame is None:
                    time.sleep(0.033)  # ~30 FPS
                    continue

                # Draw crosshairs
                height, width = frame.shape[:2]
                cv2.line(frame, (width//2, 0), (width//2, height), (0, 255, 0), 1)
                cv2.line(frame, (0, height//2), (width, height//2), (0, 255, 0), 1)

                # Add text overlay
                cv2.putText(frame, f"{self.camera_name.upper()} Camera - {width}x{height}",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                # Encode as JPEG
                ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])

                if not ret:
                    continue

                # Yield frame in multipart format
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

            except Exception as e:
                logger.error(f"Error generating frame: {e}")
                time.sleep(0.1)

    def run(self):
        """Run calibration web server"""
        print("\n" + "="*70)
        print(f"PTDTS Web-Based Camera FOV Calibration - {self.camera_name.upper()} Camera")
        print("="*70 + "\n")

        # Load config
        self.config = load_config()
        setup_logging(self.config)

        # Get camera config
        if self.camera_name == 'hq':
            cam_config = self.config.cameras.hq
        elif self.camera_name == 'gs':
            cam_config = self.config.cameras.gs
        else:
            print(f"ERROR: Unknown camera '{self.camera_name}'. Use 'hq' or 'gs'.")
            return

        print(f"Camera: {self.camera_name.upper()}")
        print(f"Resolution: {cam_config.resolution[0]}x{cam_config.resolution[1]}")
        print(f"Current FOV (config): {cam_config.fov_h}° H x {cam_config.fov_v}° V\n")

        # Initialize camera
        print("Initializing camera...")
        try:
            self.camera_manager = HardwareFactory.create_cameras(self.config)
            self.camera_manager.start_cameras()

            if self.camera_name == 'hq':
                self.camera = self.camera_manager.hq_camera
            else:
                self.camera = self.camera_manager.gs_camera

            print("Camera initialized.\n")
        except Exception as e:
            print(f"ERROR: Failed to initialize camera: {e}")
            print("\nRunning in simulation mode for demonstration.")
            print("For actual calibration, run on Raspberry Pi with cameras.\n")
            return

        print("="*70)
        print("WEB INTERFACE READY")
        print("="*70)
        print(f"\nOpen your web browser and navigate to:")
        print(f"\n  http://localhost:{self.port}")
        print(f"  http://<pi-ip-address>:{self.port}")
        print(f"\nPress Ctrl+C to stop the server")
        print("="*70 + "\n")

        try:
            # Run Flask app
            self.app.run(host='0.0.0.0', port=self.port, debug=False, threaded=True)
        except KeyboardInterrupt:
            print("\n\nCalibration stopped.")
        finally:
            if self.camera_manager:
                self.camera_manager.close()
            print("\nDone.\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Web-based camera FOV calibration")
    parser.add_argument('--camera', choices=['hq', 'gs'], default='hq',
                       help='Camera to calibrate (default: hq)')
    parser.add_argument('--port', type=int, default=8080,
                       help='Web server port (default: 8080)')
    args = parser.parse_args()

    try:
        calibration = WebCalibration(camera_name=args.camera, port=args.port)
        calibration.run()
    except KeyboardInterrupt:
        print("\n\nExiting...")
        sys.exit(0)
