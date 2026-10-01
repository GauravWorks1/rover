"""
Web-based Live Monitor for the Rover.

Streams the annotated camera feed (with bounding boxes) as MJPEG
and shows rover status on a dashboard.

Access from any device on the same network:
    http://<RPI_IP>:5000
"""

import time
import threading
import logging
from flask import Flask, Response, render_template_string, jsonify

logger = logging.getLogger(__name__)

# ============================================================================
#  HTML Dashboard Template
# ============================================================================

DASHBOARD_HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🤖 Rover Monitor</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            background: #0a0a0a;
            color: #e0e0e0;
            font-family: 'Segoe UI', monospace;
            overflow-x: hidden;
        }
        .header {
            background: linear-gradient(135deg, #1a1a2e, #16213e);
            padding: 12px 20px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 2px solid #0f3460;
        }
        .header h1 {
            font-size: 1.4em;
            color: #00d4ff;
        }
        .status-dot {
            width: 12px; height: 12px;
            border-radius: 50%;
            display: inline-block;
            margin-right: 8px;
            animation: pulse 1.5s infinite;
        }
        .status-dot.live { background: #00ff88; }
        .status-dot.offline { background: #ff4444; animation: none; }
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.4; }
        }
        .container {
            display: grid;
            grid-template-columns: 1fr 340px;
            gap: 15px;
            padding: 15px;
            max-height: calc(100vh - 60px);
        }
        @media (max-width: 900px) {
            .container {
                grid-template-columns: 1fr;
            }
        }
        .video-panel {
            background: #111;
            border-radius: 10px;
            overflow: hidden;
            border: 1px solid #222;
            position: relative;
        }
        .video-panel img {
            width: 100%;
            display: block;
        }
        .video-label {
            position: absolute;
            top: 10px;
            left: 10px;
            background: rgba(0,0,0,0.7);
            color: #00ff88;
            padding: 4px 12px;
            border-radius: 4px;
            font-size: 0.85em;
            font-weight: bold;
        }
        .side-panel {
            display: flex;
            flex-direction: column;
            gap: 12px;
        }
        .card {
            background: #161622;
            border-radius: 10px;
            padding: 16px;
            border: 1px solid #252540;
        }
        .card h3 {
            color: #00d4ff;
            font-size: 0.9em;
            margin-bottom: 12px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        .stat-row {
            display: flex;
            justify-content: space-between;
            padding: 6px 0;
            border-bottom: 1px solid #1e1e35;
            font-size: 0.95em;
        }
        .stat-row:last-child { border-bottom: none; }
        .stat-label { color: #888; }
        .stat-value { font-weight: bold; color: #fff; }
        .stat-value.mode-rc { color: #ffaa00; }
        .stat-value.mode-follow { color: #00ff88; }
        .stat-value.mode-failsafe { color: #ff4444; }

        .bar-container {
            width: 120px;
            height: 14px;
            background: #0a0a15;
            border-radius: 7px;
            overflow: hidden;
            display: inline-block;
            vertical-align: middle;
        }
        .bar-fill {
            height: 100%;
            border-radius: 7px;
            transition: width 0.3s;
        }
        .bar-speed { background: linear-gradient(90deg, #00d4ff, #0088ff); }
        .bar-steer { background: linear-gradient(90deg, #ff8800, #ffaa00); }

        .controls {
            display: flex;
            gap: 8px;
            flex-wrap: wrap;
        }
        .btn {
            padding: 8px 16px;
            border-radius: 6px;
            border: 1px solid #333;
            background: #1a1a2e;
            color: #ddd;
            cursor: pointer;
            font-size: 0.85em;
            transition: all 0.2s;
        }
        .btn:hover { background: #252545; border-color: #00d4ff; }
        .btn.active { background: #0f3460; border-color: #00d4ff; color: #00d4ff; }

        .detection-info {
            font-size: 0.85em;
            padding: 8px;
            background: #0a0a15;
            border-radius: 6px;
            margin-top: 8px;
        }
        .log-area {
            font-family: monospace;
            font-size: 0.75em;
            background: #0a0a15;
            padding: 10px;
            border-radius: 6px;
            max-height: 150px;
            overflow-y: auto;
            color: #888;
            line-height: 1.6;
        }
        .log-line.info { color: #00d4ff; }
        .log-line.warn { color: #ffaa00; }
        .log-line.error { color: #ff4444; }
    </style>
</head>
<body>
    <div class="header">
        <h1>🤖 Rover Live Monitor</h1>
        <div>
            <span class="status-dot live" id="statusDot"></span>
            <span id="connStatus">Connected</span>
        </div>
    </div>

    <div class="container">
        <div class="video-panel">
            <div class="video-label">📷 LIVE — <span id="fpsDisplay">0</span> FPS</div>
            <img id="videoFeed" src="/video_feed" alt="Camera Feed">
        </div>

        <div class="side-panel">
            <div class="card">
                <h3>⚡ Status</h3>
                <div class="stat-row">
                    <span class="stat-label">Mode</span>
                    <span class="stat-value" id="modeDisplay">—</span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">Speed</span>
                    <span class="stat-value">
                        <span id="speedValue">0</span>
                        <div class="bar-container">
                            <div class="bar-fill bar-speed" id="speedBar" style="width:50%"></div>
                        </div>
                    </span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">Steering</span>
                    <span class="stat-value">
                        <span id="steerValue">0</span>
                        <div class="bar-container">
                            <div class="bar-fill bar-steer" id="steerBar" style="width:50%"></div>
                        </div>
                    </span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">Arduino</span>
                    <span class="stat-value" id="arduinoStatus">—</span>
                </div>
            </div>

            <div class="card">
                <h3>🎯 Detection</h3>
                <div class="stat-row">
                    <span class="stat-label">Target</span>
                    <span class="stat-value" id="targetStatus">No target</span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">Confidence</span>
                    <span class="stat-value" id="confidence">—</span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">Position</span>
                    <span class="stat-value" id="position">—</span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">Distance</span>
                    <span class="stat-value" id="distance">—</span>
                </div>
            </div>

            <div class="card">
                <h3>📻 RC Channels</h3>
                <div class="stat-row">
                    <span class="stat-label">CH1 Steer</span>
                    <span class="stat-value" id="ch1">1500</span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">CH2 Throttle</span>
                    <span class="stat-value" id="ch2">1500</span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">CH3 Aux</span>
                    <span class="stat-value" id="ch3">1500</span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">SwB Mode</span>
                    <span class="stat-value" id="ch5">1000</span>
                </div>
            </div>

            <div class="card">
                <h3>📋 Log</h3>
                <div class="log-area" id="logArea"></div>
            </div>
        </div>
    </div>

    <script>
        // Poll status every 500ms
        function updateStatus() {
            fetch('/api/status')
                .then(r => r.json())
                .then(data => {
                    // Mode
                    const modeEl = document.getElementById('modeDisplay');
                    const modeNames = {0: 'RC', 1: 'FOLLOW', 2: 'FAILSAFE'};
                    const modeClasses = {0: 'mode-rc', 1: 'mode-follow', 2: 'mode-failsafe'};
                    modeEl.textContent = modeNames[data.mode] || 'UNKNOWN';
                    modeEl.className = 'stat-value ' + (modeClasses[data.mode] || '');

                    // Speed & Steer
                    document.getElementById('speedValue').textContent = data.speed;
                    document.getElementById('steerValue').textContent = data.steer;
                    const speedPct = Math.min(100, Math.abs(data.speed) / 2.55);
                    const steerPct = (data.steer + 100) / 2;
                    document.getElementById('speedBar').style.width = speedPct + '%';
                    document.getElementById('steerBar').style.width = steerPct + '%';

                    // Arduino
                    document.getElementById('arduinoStatus').textContent =
                        data.arduino_connected ? '✅ Connected' : '❌ Disconnected';

                    // Detection
                    if (data.detection) {
                        document.getElementById('targetStatus').textContent = '🟢 Locked';
                        document.getElementById('targetStatus').style.color = '#00ff88';
                        document.getElementById('confidence').textContent =
                            (data.detection.confidence * 100).toFixed(0) + '%';
                        document.getElementById('position').textContent =
                            '(' + data.detection.cx + ', ' + data.detection.cy + ')';
                        document.getElementById('distance').textContent =
                            (data.detection.area_ratio * 100).toFixed(1) + '% frame';
                    } else {
                        document.getElementById('targetStatus').textContent = '🔴 No target';
                        document.getElementById('targetStatus').style.color = '#ff4444';
                        document.getElementById('confidence').textContent = '—';
                        document.getElementById('position').textContent = '—';
                        document.getElementById('distance').textContent = '—';
                    }

                    // RC Channels
                    document.getElementById('ch1').textContent = data.rc_channels[0];
                    document.getElementById('ch2').textContent = data.rc_channels[1];
                    document.getElementById('ch3').textContent = data.rc_channels[2];
                    document.getElementById('ch5').textContent = data.rc_channels[3];

                    // FPS
                    document.getElementById('fpsDisplay').textContent = data.fps.toFixed(1);

                    // Connection status
                    document.getElementById('statusDot').className = 'status-dot live';
                    document.getElementById('connStatus').textContent = 'Connected';
                })
                .catch(err => {
                    document.getElementById('statusDot').className = 'status-dot offline';
                    document.getElementById('connStatus').textContent = 'Disconnected';
                });
        }

        // Poll logs
        function updateLogs() {
            fetch('/api/logs')
                .then(r => r.json())
                .then(data => {
                    const logArea = document.getElementById('logArea');
                    logArea.innerHTML = data.logs.map(l => {
                        let cls = 'info';
                        if (l.includes('WARNING') || l.includes('WARN')) cls = 'warn';
                        if (l.includes('ERROR')) cls = 'error';
                        return '<div class="log-line ' + cls + '">' + l + '</div>';
                    }).join('');
                    logArea.scrollTop = logArea.scrollHeight;
                })
                .catch(() => {});
        }

        setInterval(updateStatus, 500);
        setInterval(updateLogs, 2000);
        updateStatus();
        updateLogs();

        // Reload video feed if it stalls
        const videoEl = document.getElementById('videoFeed');
        let lastCheck = Date.now();
        videoEl.onerror = function() {
            setTimeout(() => {
                videoEl.src = '/video_feed?' + Date.now();
            }, 2000);
        };
    </script>
</body>
</html>
"""


class WebMonitor:
    """
    Flask-based web monitor for the rover.

    Provides:
      - MJPEG live camera stream with detection overlay
      - JSON API for rover status
      - Dashboard UI
    """

    def __init__(self, rover_controller, host='0.0.0.0', port=5000):
        self.rover = rover_controller
        self.host = host
        self.port = port
        self.app = Flask(__name__)

        # Log buffer (last N lines)
        self._log_buffer = []
        self._log_max = 50
        self._log_lock = threading.Lock()

        # Install custom log handler to capture logs
        self._setup_log_capture()

        # Register routes
        self._register_routes()

        # Current command values (updated by rover controller)
        self.current_speed = 0
        self.current_steer = 0

    def _setup_log_capture(self):
        """Add a handler that captures log lines for the web UI."""
        handler = _WebLogHandler(self)
        handler.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s [%(levelname)-5s] %(message)s',
                                       datefmt='%H:%M:%S')
        handler.setFormatter(formatter)
        logging.getLogger().addHandler(handler)

    def add_log(self, message):
        """Add a log line to the buffer."""
        with self._log_lock:
            self._log_buffer.append(message)
            if len(self._log_buffer) > self._log_max:
                self._log_buffer = self._log_buffer[-self._log_max:]

    def _register_routes(self):
        """Register Flask URL routes."""

        @self.app.route('/')
        def index():
            return render_template_string(DASHBOARD_HTML)

        @self.app.route('/video_feed')
        def video_feed():
            return Response(
                self._generate_frames(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )

        @self.app.route('/api/status')
        def api_status():
            # Get detection info
            detection = None
            det_age = float('inf')
            if not self.rover.no_camera and self.rover.detector.is_running():
                det_raw, det_age = self.rover.detector.get_detection()
                if det_raw and det_age < 2.0:
                    detection = {
                        'cx': int(det_raw['cx']),
                        'cy': int(det_raw['cy']),
                        'w': int(det_raw['w']),
                        'h': int(det_raw['h']),
                        'area_ratio': round(float(det_raw['area_ratio']), 4),
                        'confidence': round(float(det_raw['confidence']), 3)
                    }

            return jsonify({
                'mode': self.rover.serial.get_mode(),
                'speed': self.current_speed,
                'steer': self.current_steer,
                'arduino_connected': self.rover.serial.is_connected(),
                'rc_channels': self.rover.serial.get_rc_channels(),
                'detection': detection,
                'fps': self.rover.detector.get_fps() if not self.rover.no_camera else 0,
                'target_acquired': self.rover.follower.target_acquired
            })

        @self.app.route('/api/logs')
        def api_logs():
            with self._log_lock:
                return jsonify({'logs': list(self._log_buffer)})

    def _generate_frames(self):
        """Generator that yields MJPEG frames for the video stream."""
        import cv2

        while True:
            frame = None
            if not self.rover.no_camera and self.rover.detector.is_running():
                frame = self.rover.detector.get_annotated_frame()

            if frame is not None:
                # Add status overlay to frame
                h, w = frame.shape[:2]
                mode_names = {0: 'RC', 1: 'FOLLOW', 2: 'FAILSAFE'}
                mode = self.rover.serial.get_mode()
                mode_name = mode_names.get(mode, '?')
                mode_colors = {0: (0, 170, 255), 1: (0, 255, 100), 2: (0, 0, 255)}
                color = mode_colors.get(mode, (255, 255, 255))

                # Mode label
                cv2.putText(frame, f"MODE: {mode_name}", (w - 200, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                # Speed/Steer
                cv2.putText(frame, f"SPD: {self.current_speed:+4d}", (w - 200, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                cv2.putText(frame, f"STR: {self.current_steer:+4d}", (w - 200, 85),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

                # Encode as JPEG
                ret, buffer = cv2.imencode('.jpg', frame,
                                           [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' +
                           buffer.tobytes() + b'\r\n')
            else:
                # No frame available — send a black placeholder
                import numpy as np
                blank = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(blank, "No Camera Feed", (180, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 100, 100), 2)
                ret, buffer = cv2.imencode('.jpg', blank)
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' +
                           buffer.tobytes() + b'\r\n')

            time.sleep(0.08)  # ~12 FPS stream rate

    def start(self):
        """Start the web server in a background thread."""
        thread = threading.Thread(target=self._run_server, daemon=True)
        thread.start()
        logger.info(f"📺 Web monitor started at http://0.0.0.0:{self.port}")
        logger.info(f"   Open in browser: http://<RPI_IP>:{self.port}")

    def _run_server(self):
        """Run Flask in a background thread."""
        # Suppress Flask's default logging
        import logging as _logging
        _logging.getLogger('werkzeug').setLevel(_logging.WARNING)

        self.app.run(
            host=self.host,
            port=self.port,
            debug=False,
            threaded=True,
            use_reloader=False
        )


class _WebLogHandler(logging.Handler):
    """Custom log handler that feeds log lines to the WebMonitor."""

    def __init__(self, monitor):
        super().__init__()
        self.monitor = monitor

    def emit(self, record):
        try:
            msg = self.format(record)
            self.monitor.add_log(msg)
        except Exception:
            pass
