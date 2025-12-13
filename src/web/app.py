"""
Flask web application for PTDTS
Real-time monitoring and control interface
"""

import os
import io
import time
import logging
import threading
from typing import Optional, Dict, Any
from flask import Flask, render_template, Response, jsonify, request
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import cv2
import numpy as np

logger = logging.getLogger(__name__)


class PTDTSWebApp:
    """
    Web application for PTDTS monitoring and control
    """

    def __init__(self, state_machine, pan_controller, tilt_controller,
                 camera_manager, visual_detector, config, system_monitor=None,
                 gamepad_controller=None):
        """
        Initialize web application

        Args:
            state_machine: System state machine
            pan_controller: Pan axis controller
            tilt_controller: Tilt axis controller
            camera_manager: Camera manager
            visual_detector: Visual detector
            config: Configuration object
            system_monitor: System monitor (optional)
            gamepad_controller: Gamepad controller (optional)
        """
        self.state_machine = state_machine
        self.pan_controller = pan_controller
        self.tilt_controller = tilt_controller
        self.camera_manager = camera_manager
        self.visual_detector = visual_detector
        self.config = config
        self.system_monitor = system_monitor
        self.gamepad_controller = gamepad_controller

        # Flask app setup
        template_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'templates')
        static_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'static')

        self.app = Flask(__name__,
                        template_folder=template_dir,
                        static_folder=static_dir)
        self.app.config['SECRET_KEY'] = 'ptdts-secret-key-change-in-production'

        # Enable CORS
        CORS(self.app)

        # SocketIO setup
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", async_mode='threading')

        # Telemetry broadcast state
        self.telemetry_thread: Optional[threading.Thread] = None
        self.telemetry_running = False
        self.last_frame: Optional[np.ndarray] = None
        self.last_frame_lock = threading.Lock()

        # Setup routes
        self._setup_routes()
        self._setup_socketio()

        logger.info("Web application initialized")

    def _setup_routes(self):
        """Setup Flask routes"""

        @self.app.route('/')
        def index():
            """Main dashboard page"""
            return render_template('index.html')

        @self.app.route('/api/status')
        def api_status():
            """Get current system status"""
            try:
                state_info = self.state_machine.get_state_info()
                pan_status = self.pan_controller.get_status()
                tilt_status = self.tilt_controller.get_status()

                return jsonify({
                    'status': 'ok',
                    'state': state_info,
                    'pan': pan_status,
                    'tilt': tilt_status,
                    'timestamp': time.time()
                })
            except Exception as e:
                logger.error(f"Error getting status: {e}")
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/api/config')
        def api_config():
            """Get system configuration"""
            try:
                return jsonify({
                    'status': 'ok',
                    'config': {
                        'simulation_mode': self.config.simulation_mode,
                        'pan_limits': {
                            'max_velocity': self.config.pan_motor.max_velocity_dps,
                            'max_acceleration': self.config.pan_motor.max_acceleration_dps2
                        },
                        'tilt_limits': {
                            'min_angle': self.config.tilt_servo.min_angle,
                            'max_angle': self.config.tilt_servo.max_angle
                        },
                        'cameras': {
                            'hq': {
                                'resolution': self.config.cameras.hq.resolution,
                                'framerate': self.config.cameras.hq.framerate
                            },
                            'gs': {
                                'resolution': self.config.cameras.gs.resolution,
                                'framerate': self.config.cameras.gs.framerate
                            }
                        }
                    }
                })
            except Exception as e:
                logger.error(f"Error getting config: {e}")
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/api/hardware/monitor')
        def api_hardware_monitor():
            """Get comprehensive hardware monitoring data"""
            try:
                # Collect all hardware monitoring metrics
                data = {
                    'timestamp': time.time(),
                    'system': {},
                    'motor': {},
                    'cameras': {},
                    'control_loops': {},
                    'communication': {}
                }

                # System metrics
                if self.system_monitor:
                    data['system'] = self.system_monitor.get_metrics()

                # Motor metrics
                pan_status = self.pan_controller.get_status()
                if 'motor' in pan_status:
                    data['motor'] = pan_status['motor']

                # Camera metrics
                if hasattr(self.camera_manager.hq_camera, 'get_performance_metrics'):
                    data['cameras']['hq'] = self.camera_manager.hq_camera.get_performance_metrics()
                if hasattr(self.camera_manager.gs_camera, 'get_performance_metrics'):
                    data['cameras']['gs'] = self.camera_manager.gs_camera.get_performance_metrics()

                # Control loop timing
                if 'timing' in pan_status:
                    data['control_loops']['pan'] = pan_status['timing']

                # Communication errors
                if hasattr(self.pan_controller.encoder, 'get_error_status'):
                    data['communication']['encoder_spi'] = self.pan_controller.encoder.get_error_status()
                if hasattr(self.tilt_controller.servo, 'adc'):
                    if hasattr(self.tilt_controller.servo.adc, 'get_error_status'):
                        data['communication']['servo_adc_i2c'] = self.tilt_controller.servo.adc.get_error_status()

                return jsonify({
                    'status': 'ok',
                    'data': data
                })
            except Exception as e:
                logger.error(f"Error getting hardware monitor data: {e}")
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/api/control/manual', methods=['POST'])
        def api_manual_mode():
            """Toggle manual mode"""
            try:
                data = request.json
                enable = data.get('enable', False)

                if enable:
                    self.state_machine.request_manual_mode()
                else:
                    self.state_machine.request_auto_mode()

                return jsonify({'status': 'ok', 'manual_mode': enable})
            except Exception as e:
                logger.error(f"Error toggling manual mode: {e}")
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/api/control/pan', methods=['POST'])
        def api_pan_control():
            """Pan axis manual control"""
            try:
                data = request.json
                velocity = data.get('velocity', 0.0)

                # Only allow in manual mode
                if self.state_machine.get_current_state().value != 'manual':
                    return jsonify({'status': 'error', 'message': 'Not in manual mode'}), 400

                self.pan_controller.set_manual_velocity(velocity)
                return jsonify({'status': 'ok', 'velocity': velocity})
            except Exception as e:
                logger.error(f"Error controlling pan: {e}")
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/api/control/tilt', methods=['POST'])
        def api_tilt_control():
            """Tilt axis manual control"""
            try:
                data = request.json
                angle = data.get('angle', 90.0)

                # Only allow in manual mode
                if self.state_machine.get_current_state().value != 'manual':
                    return jsonify({'status': 'error', 'message': 'Not in manual mode'}), 400

                self.tilt_controller.set_target_angle(angle)
                return jsonify({'status': 'ok', 'angle': angle})
            except Exception as e:
                logger.error(f"Error controlling tilt: {e}")
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/api/gamepad/status')
        def api_gamepad_status():
            """Get gamepad controller status"""
            try:
                if not self.gamepad_controller:
                    return jsonify({
                        'status': 'ok',
                        'enabled': False,
                        'connected': False,
                        'message': 'Gamepad not initialized'
                    })

                gamepad_state = self.gamepad_controller.get_state_dict()
                return jsonify({
                    'status': 'ok',
                    'enabled': True,
                    **gamepad_state
                })
            except Exception as e:
                logger.error(f"Error getting gamepad status: {e}")
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/api/state/set', methods=['POST'])
        def api_set_state():
            """Manually set system state"""
            try:
                data = request.json
                state_name = data.get('state')

                if not state_name:
                    return jsonify({'status': 'error', 'message': 'State name required'}), 400

                # Get list of valid states
                from src.state.state_machine import SystemState
                valid_states = [s.value for s in SystemState]

                if state_name.lower() not in valid_states:
                    return jsonify({
                        'status': 'error',
                        'message': f'Invalid state. Valid states: {valid_states}'
                    }), 400

                # Request state change
                self.state_machine.request_state(state_name)

                return jsonify({
                    'status': 'ok',
                    'state': state_name,
                    'message': f'Transitioned to {state_name} state'
                })
            except Exception as e:
                logger.error(f"Error setting state: {e}")
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/api/state/list')
        def api_list_states():
            """Get list of available system states"""
            try:
                from src.state.state_machine import SystemState
                states = [s.value for s in SystemState]
                current_state = self.state_machine.get_current_state().value

                return jsonify({
                    'status': 'ok',
                    'states': states,
                    'current': current_state
                })
            except Exception as e:
                logger.error(f"Error listing states: {e}")
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/api/camera/switch', methods=['POST'])
        def api_switch_camera():
            """Manually switch active camera"""
            try:
                data = request.json
                camera_name = data.get('camera')

                if not camera_name:
                    return jsonify({'status': 'error', 'message': 'Camera name required'}), 400

                if camera_name.lower() not in ['hq', 'gs']:
                    return jsonify({
                        'status': 'error',
                        'message': 'Invalid camera. Must be "hq" or "gs"'
                    }), 400

                # Switch camera with manual override
                self.camera_manager.set_manual_camera(camera_name)

                return jsonify({
                    'status': 'ok',
                    'camera': camera_name,
                    'message': f'Switched to {camera_name.upper()} camera (manual override enabled)'
                })
            except Exception as e:
                logger.error(f"Error switching camera: {e}")
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/api/camera/auto', methods=['POST'])
        def api_camera_auto():
            """Disable manual camera override and return to automatic switching"""
            try:
                self.camera_manager.disable_manual_override()

                return jsonify({
                    'status': 'ok',
                    'message': 'Manual camera override disabled, automatic switching resumed'
                })
            except Exception as e:
                logger.error(f"Error disabling camera override: {e}")
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/api/camera/status')
        def api_camera_status():
            """Get camera status"""
            try:
                return jsonify({
                    'status': 'ok',
                    'active_camera': self.camera_manager.get_active_camera_name(),
                    'manual_override': self.camera_manager.is_manual_override_active()
                })
            except Exception as e:
                logger.error(f"Error getting camera status: {e}")
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/video_feed')
        def video_feed():
            """Video streaming route"""
            return Response(
                self._generate_frames(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )

    def _setup_socketio(self):
        """Setup SocketIO event handlers"""

        @self.socketio.on('connect')
        def handle_connect():
            """Handle client connection"""
            logger.info(f"Client connected: {request.sid}")
            emit('connected', {'status': 'ok'})

        @self.socketio.on('disconnect')
        def handle_disconnect():
            """Handle client disconnection"""
            logger.info(f"Client disconnected: {request.sid}")

        @self.socketio.on('request_telemetry')
        def handle_telemetry_request(data):
            """Handle telemetry data request"""
            try:
                telemetry = self._get_telemetry()
                emit('telemetry', telemetry)
            except Exception as e:
                logger.error(f"Error sending telemetry: {e}")
                emit('error', {'message': str(e)})

    def _get_telemetry(self) -> Dict[str, Any]:
        """
        Get current telemetry data

        Returns:
            Dictionary with telemetry data
        """
        try:
            state_info = self.state_machine.get_state_info()
            pan_status = self.pan_controller.get_status()
            tilt_status = self.tilt_controller.get_status()

            # Get visual detector stats
            detector_stats = {
                'fps': getattr(self.visual_detector, 'fps', 0.0),
                'inference_time': getattr(self.visual_detector, 'inference_time_ms', 0.0)
            }

            # Get camera performance metrics
            camera_metrics = {}
            if hasattr(self.camera_manager.hq_camera, 'get_performance_metrics'):
                camera_metrics['hq'] = self.camera_manager.hq_camera.get_performance_metrics()
            if hasattr(self.camera_manager.gs_camera, 'get_performance_metrics'):
                camera_metrics['gs'] = self.camera_manager.gs_camera.get_performance_metrics()

            # Get system metrics
            system_metrics = {}
            if self.system_monitor:
                system_metrics = self.system_monitor.get_metrics()

            # Get motor/encoder fault status
            hardware_status = {}

            # Motor status (includes current and fault)
            if 'motor' in pan_status:
                hardware_status['motor'] = pan_status['motor']

            # Encoder error status
            if hasattr(self.pan_controller.encoder, 'get_error_status'):
                hardware_status['encoder'] = self.pan_controller.encoder.get_error_status()

            # Servo ADC error status
            if hasattr(self.tilt_controller.servo, 'adc'):
                if hasattr(self.tilt_controller.servo.adc, 'get_error_status'):
                    hardware_status['servo_adc'] = self.tilt_controller.servo.adc.get_error_status()

            # Get gamepad status
            gamepad_status = {}
            if self.gamepad_controller:
                try:
                    gamepad_status = {
                        'connected': self.gamepad_controller.is_connected(),
                        'pan_command': self.gamepad_controller.get_pan_velocity_command(),
                        'tilt_command': self.gamepad_controller.get_tilt_angle_command()
                    }
                except Exception as e:
                    logger.debug(f"Error getting gamepad status: {e}")
                    gamepad_status = {'connected': False, 'error': str(e)}

            telemetry = {
                'timestamp': time.time(),
                'state': {
                    'current': state_info['state'],
                    'duration': state_info['state_duration'],
                    'target': state_info.get('current_target'),
                    'pending_acoustic': state_info.get('pending_acoustic')
                },
                'pan': {
                    'position': pan_status.get('position_deg', 0.0),
                    'velocity': pan_status.get('velocity_dps', 0.0),
                    'mode': pan_status.get('mode', 'idle'),
                    'target_position': pan_status.get('target_position', None),
                    'target_velocity': pan_status.get('target_velocity', None),
                    'timing': pan_status.get('timing', {})
                },
                'tilt': {
                    'angle': tilt_status.get('current_angle', 90.0),
                    'target_angle': tilt_status.get('target_angle', 90.0)
                },
                'detector': detector_stats,
                'camera': {
                    'active': self.camera_manager.get_active_camera_name() if hasattr(self.camera_manager, 'get_active_camera_name') else 'hq',
                    'performance': camera_metrics
                },
                'system': system_metrics,
                'hardware': hardware_status,
                'gamepad': gamepad_status
            }

            return telemetry

        except Exception as e:
            logger.error(f"Error getting telemetry: {e}")
            return {'error': str(e)}

    def _generate_frames(self):
        """
        Generate frames for video streaming

        Yields:
            JPEG frames in multipart format
        """
        while True:
            try:
                # Capture frame
                frame, camera_name = self.camera_manager.capture_frame()

                if frame is None:
                    time.sleep(0.033)  # ~30 FPS
                    continue

                # Run detection and draw overlays
                if self.visual_detector is not None:
                    detections = self.visual_detector.detect(frame)
                    annotated_frame = self.visual_detector.draw_detections(frame, detections)
                else:
                    # No detector available, use original frame
                    annotated_frame = frame

                # Encode as JPEG
                ret, buffer = cv2.imencode('.jpg', annotated_frame,
                                          [cv2.IMWRITE_JPEG_QUALITY, self.config.web.video['quality']])

                if not ret:
                    continue

                frame_bytes = buffer.tobytes()

                # Yield frame in multipart format
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

                # Limit FPS
                time.sleep(1.0 / self.config.web.video['max_fps'])

            except Exception as e:
                logger.error(f"Error generating frame: {e}")
                time.sleep(0.1)

    def start_telemetry_broadcast(self):
        """Start broadcasting telemetry to connected clients"""
        if self.telemetry_running:
            return

        self.telemetry_running = True
        self.telemetry_thread = threading.Thread(target=self._telemetry_broadcast_loop, daemon=True)
        self.telemetry_thread.start()
        logger.info("Telemetry broadcast started")

    def stop_telemetry_broadcast(self):
        """Stop broadcasting telemetry"""
        self.telemetry_running = False
        if self.telemetry_thread:
            self.telemetry_thread.join(timeout=2.0)
        logger.info("Telemetry broadcast stopped")

    def _telemetry_broadcast_loop(self):
        """Telemetry broadcast loop"""
        rate = self.config.web.telemetry_rate_hz
        period = 1.0 / rate

        while self.telemetry_running:
            try:
                telemetry = self._get_telemetry()
                self.socketio.emit('telemetry', telemetry)
                time.sleep(period)
            except Exception as e:
                logger.error(f"Error in telemetry broadcast: {e}")
                time.sleep(1.0)

    def run(self, host: str = None, port: int = None, debug: bool = False):
        """
        Run the web application

        Args:
            host: Host address (default from config)
            port: Port number (default from config)
            debug: Debug mode
        """
        host = host or self.config.web.host
        port = port or self.config.web.port
        debug = debug or self.config.web.debug

        # Start telemetry broadcast
        self.start_telemetry_broadcast()

        logger.info(f"Starting web server on {host}:{port}")

        try:
            self.socketio.run(self.app, host=host, port=port, debug=debug, allow_unsafe_werkzeug=True)
        except KeyboardInterrupt:
            logger.info("Shutting down web server")
        finally:
            self.stop_telemetry_broadcast()


def create_app(state_machine, pan_controller, tilt_controller,
               camera_manager, visual_detector, config, system_monitor=None,
               gamepad_controller=None):
    """
    Factory function to create PTDTS web app

    Args:
        state_machine: System state machine
        pan_controller: Pan axis controller
        tilt_controller: Tilt axis controller
        camera_manager: Camera manager
        visual_detector: Visual detector
        config: Configuration object
        system_monitor: System monitor (optional)
        gamepad_controller: Gamepad controller (optional)

    Returns:
        PTDTSWebApp instance
    """
    return PTDTSWebApp(state_machine, pan_controller, tilt_controller,
                       camera_manager, visual_detector, config, system_monitor,
                       gamepad_controller)
