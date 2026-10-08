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
from flask import Flask, Response, render_template_string, jsonify, request

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
    <title>🤖 Rover Monitor & Remote Control</title>
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
            grid-template-columns: 1fr 370px;
            gap: 15px;
            padding: 15px;
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
            display: flex;
            justify-content: space-between;
            align-items: center;
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
        .stat-value.mode-web { color: #c084fc; }

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

        /* Web Remote Control Panel */
        .remote-card {
            border: 1px solid #3b2d64;
            background: linear-gradient(180deg, #19162b 0%, #161622 100%);
        }
        .speed-control-box {
            background: #0e0e1a;
            padding: 10px 12px;
            border-radius: 8px;
            margin-bottom: 12px;
            border: 1px solid #252545;
        }
        .speed-header {
            display: flex;
            justify-content: space-between;
            font-size: 0.85em;
            margin-bottom: 6px;
            color: #bbb;
        }
        .speed-slider {
            width: 100%;
            accent-color: #00d4ff;
            cursor: pointer;
            height: 6px;
        }
        .dpad-grid {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 8px;
            margin-bottom: 10px;
        }
        .ctrl-btn {
            padding: 14px 8px;
            border-radius: 8px;
            border: 1px solid #353560;
            background: #1e1e36;
            color: #fff;
            font-weight: bold;
            font-size: 0.85em;
            cursor: pointer;
            text-align: center;
            user-select: none;
            -webkit-user-select: none;
            touch-action: manipulation;
            transition: all 0.15s;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            gap: 4px;
        }
        .ctrl-btn span.icon { font-size: 1.35em; }
        .ctrl-btn:hover { background: #2a2a4a; border-color: #00d4ff; }
        .ctrl-btn:active, .ctrl-btn.active {
            background: #00d4ff;
            color: #000;
            border-color: #fff;
            box-shadow: 0 0 12px rgba(0, 212, 255, 0.6);
        }
        .ctrl-btn.stop-btn {
            background: #45151b;
            border-color: #ff4444;
            color: #ffaaaa;
        }
        .ctrl-btn.stop-btn:hover { background: #651a22; }
        .ctrl-btn.stop-btn:active, .ctrl-btn.stop-btn.active {
            background: #ff3333;
            color: #fff;
            box-shadow: 0 0 14px rgba(255, 51, 51, 0.8);
        }
        .mode-toggle-row {
            display: flex;
            justify-content: space-between;
            align-items: center;
            font-size: 0.8em;
            color: #aaa;
            padding-top: 6px;
            border-top: 1px solid #22223b;
        }
        .badge-web {
            font-size: 0.75em;
            padding: 2px 8px;
            border-radius: 10px;
            background: #2b2b40;
            color: #999;
        }
        .badge-web.active {
            background: #7e22ce;
            color: #fff;
        }
        .web-lock-btn {
            width: 100%;
            padding: 12px 10px;
            margin-bottom: 12px;
            border-radius: 8px;
            border: 2px solid #4b5563;
            background: #1f2937;
            color: #e5e7eb;
            font-weight: bold;
            font-size: 0.9em;
            cursor: pointer;
            transition: all 0.2s;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 8px;
        }
        .web-lock-btn:hover {
            border-color: #c084fc;
            background: #2e1065;
        }
        .web-lock-btn.locked {
            background: linear-gradient(135deg, #7e22ce, #db2777);
            border-color: #f0abfc;
            color: #ffffff;
            box-shadow: 0 0 16px rgba(192, 132, 252, 0.7);
        }

        .log-area {
            font-family: monospace;
            font-size: 0.75em;
            background: #0a0a15;
            padding: 10px;
            border-radius: 6px;
            max-height: 140px;
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
        <h1>🤖 Rover Live Monitor & Web Remote</h1>
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
            <!-- NEW: WEB REMOTE CONTROL PANEL -->
            <div class="card remote-card">
                <h3>
                    <span>🕹️ Web Remote Control</span>
                    <span class="badge-web" id="webStateBadge">IDLE</span>
                </h3>

                <!-- Master Web Mode Lock Button (Completely shuts off RC & Follow modes) -->
                <button class="web-lock-btn" id="webLockBtn" onclick="toggleWebModeLock()">
                    <span id="webLockIcon">🔓</span>
                    <span id="webLockText">ENABLE WEB MODE (Shut Off RC & Follow)</span>
                </button>

                <!-- Universal Speed Control -->
                <div class="speed-control-box">
                    <div class="speed-header">
                        <span>⚡ Universal Motor Speed</span>
                        <strong id="webSpeedLabel" style="color:#00d4ff;">60% (120 PWM)</strong>
                    </div>
                    <input type="range" id="webSpeedSlider" class="speed-slider"
                           min="10" max="100" step="5" value="60"
                           oninput="onSpeedSliderChange(this.value)">
                </div>

                <!-- Control Buttons Grid -->
                <div class="dpad-grid">
                    <div></div>
                    <button class="ctrl-btn" id="btn-forward"
                            onmousedown="handleBtnPress('forward')" onmouseup="handleBtnRelease()" onmouseleave="handleBtnRelease()"
                            ontouchstart="handleTouchStart(event, 'forward')" ontouchend="handleTouchEnd(event)">
                        <span class="icon">▲</span>
                        <span>FORWARD</span>
                    </button>
                    <div></div>

                    <button class="ctrl-btn" id="btn-rotate_left"
                            onmousedown="handleBtnPress('rotate_left')" onmouseup="handleBtnRelease()" onmouseleave="handleBtnRelease()"
                            ontouchstart="handleTouchStart(event, 'rotate_left')" ontouchend="handleTouchEnd(event)">
                        <span class="icon">↺</span>
                        <span>360° LEFT</span>
                    </button>

                    <button class="ctrl-btn stop-btn" id="btn-stop"
                            onclick="triggerWebStop()">
                        <span class="icon">⏹</span>
                        <span>STOP</span>
                    </button>

                    <button class="ctrl-btn" id="btn-rotate_right"
                            onmousedown="handleBtnPress('rotate_right')" onmouseup="handleBtnRelease()" onmouseleave="handleBtnRelease()"
                            ontouchstart="handleTouchStart(event, 'rotate_right')" ontouchend="handleTouchEnd(event)">
                        <span class="icon">↻</span>
                        <span>360° RIGHT</span>
                    </button>

                    <div></div>
                    <button class="ctrl-btn" id="btn-reverse"
                            onmousedown="handleBtnPress('reverse')" onmouseup="handleBtnRelease()" onmouseleave="handleBtnRelease()"
                            ontouchstart="handleTouchStart(event, 'reverse')" ontouchend="handleTouchEnd(event)">
                        <span class="icon">▼</span>
                        <span>REVERSE</span>
                    </button>
                    <div></div>
                </div>

                <div class="mode-toggle-row">
                    <label style="cursor:pointer; display:flex; align-items:center; gap:6px;">
                        <input type="checkbox" id="latchModeToggle" checked>
                        <span>Latch Mode (Click to run, click STOP to halt)</span>
                    </label>
                </div>
            </div>

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
                <h3>🎯 Detection & Smart Follow</h3>
                <div class="stat-row">
                    <span class="stat-label">Target</span>
                    <span class="stat-value" id="targetStatus">No target</span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">Owner Shirt Lock</span>
                    <span class="stat-value" id="ownerLockStatus">—</span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">Position</span>
                    <span class="stat-value" id="position">—</span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">Distance</span>
                    <span class="stat-value" id="distance">—</span>
                </div>
                <div class="speed-slider-box" style="margin-top:10px; margin-bottom:10px; border-color:rgba(0, 255, 136, 0.25);">
                    <div class="speed-header">
                        <span class="stat-label">Follow Mode Speed (Only)</span>
                        <span class="stat-value" id="followSpeedLabel" style="color:#00ff88;">20 RPM (68 PWM)</span>
                    </div>
                    <input type="range" id="followSpeedSlider" class="speed-slider"
                           min="5" max="60" step="1" value="20"
                           oninput="onFollowSpeedChange(this.value)">
                </div>
                <div style="display:grid; grid-template-columns:1fr 1fr; gap:6px; margin-top:6px;">
                    <button class="ctrl-btn" style="padding:8px 6px; font-size:0.75em;" onclick="sendFollowAction('lock_owner')">
                        👕 Re-Lock Shirt
                    </button>
                    <button class="ctrl-btn" style="padding:8px 6px; font-size:0.75em;" onclick="sendFollowAction('reset_owner')">
                        🔓 Reset Owner
                    </button>
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
        // =====================================================================
        // Web Remote Control Logic
        // =====================================================================
        let webModeLocked = false;
        let activeWebAction = null;
        let webCmdTimer = null;
        let currentSpeedPct = 60;

        function onSpeedSliderChange(val) {
            currentSpeedPct = parseInt(val, 10);
            const pwm = Math.round((currentSpeedPct / 100) * 200);
            document.getElementById('webSpeedLabel').textContent =
                currentSpeedPct + '% (' + pwm + ' PWM)';
            // If currently moving, immediately send updated speed
            if (activeWebAction) {
                sendWebControlCommand(activeWebAction);
            }
        }

        function updateLockButtonUI() {
            const btn = document.getElementById('webLockBtn');
            const icon = document.getElementById('webLockIcon');
            const txt = document.getElementById('webLockText');
            if (webModeLocked) {
                btn.classList.add('locked');
                icon.textContent = '🔒';
                txt.textContent = 'WEB MODE ACTIVE — Click to Return to RC/Follow';
            } else {
                btn.classList.remove('locked');
                icon.textContent = '🔓';
                txt.textContent = 'ENABLE WEB MODE (Shut Off RC & Follow)';
            }
        }

        function toggleWebModeLock() {
            const targetLock = !webModeLocked;
            if (!targetLock) {
                activeWebAction = null;
                if (webCmdTimer) {
                    clearInterval(webCmdTimer);
                    webCmdTimer = null;
                }
            }
            fetch('/api/web_lock', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({locked: targetLock})
            })
            .then(r => r.json())
            .then(data => {
                webModeLocked = !!data.web_locked;
                updateLockButtonUI();
                updateRemoteUI();
            })
            .catch(() => {});
        }

        function sendWebControlCommand(action) {
            fetch('/api/web_control', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    action: action,
                    speed_pct: currentSpeedPct
                })
            })
            .then(r => r.json())
            .then(data => {
                if (data.web_locked !== undefined) {
                    webModeLocked = !!data.web_locked;
                    updateLockButtonUI();
                }
            })
            .catch(() => {});
        }

        function updateRemoteUI() {
            const actions = ['forward', 'reverse', 'rotate_left', 'rotate_right'];
            actions.forEach(a => {
                const btn = document.getElementById('btn-' + a);
                if (btn) {
                    btn.classList.toggle('active', activeWebAction === a);
                }
            });
            const badge = document.getElementById('webStateBadge');
            if (activeWebAction) {
                badge.textContent = activeWebAction.replace('_', ' ').toUpperCase();
                badge.classList.add('active');
            } else if (webModeLocked) {
                badge.textContent = 'LOCKED (RC/FOLLOW OFF)';
                badge.classList.add('active');
            } else {
                badge.textContent = 'IDLE (RC / FOLLOW)';
                badge.classList.remove('active');
            }
        }

        function startWebAction(action) {
            activeWebAction = action;
            webModeLocked = true; // Auto-lock Web Mode so RC & Follow cannot interfere
            updateLockButtonUI();
            updateRemoteUI();
            sendWebControlCommand(action);
            if (webCmdTimer) clearInterval(webCmdTimer);
            // Send keepalive every 200ms while action is active
            webCmdTimer = setInterval(() => {
                if (activeWebAction) {
                    sendWebControlCommand(activeWebAction);
                }
            }, 200);
        }

        function triggerWebStop() {
            activeWebAction = null;
            if (webCmdTimer) {
                clearInterval(webCmdTimer);
                webCmdTimer = null;
            }
            updateRemoteUI();
            sendWebControlCommand('stop');
        }

        function handleBtnPress(action) {
            const isLatch = document.getElementById('latchModeToggle').checked;
            if (isLatch) {
                if (activeWebAction === action) {
                    triggerWebStop();
                } else {
                    startWebAction(action);
                }
            } else {
                startWebAction(action);
            }
        }

        function handleBtnRelease() {
            const isLatch = document.getElementById('latchModeToggle').checked;
            if (!isLatch && activeWebAction) {
                triggerWebStop();
            }
        }

        function handleTouchStart(e, action) {
            e.preventDefault();
            handleBtnPress(action);
        }

        function handleTouchEnd(e) {
            e.preventDefault();
            handleBtnRelease();
        }

        function sendFollowAction(action) {
            fetch('/api/follow_action', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: action})
            })
            .then(() => updateStatus())
            .catch(() => {});
        }

        let followSpeedEditing = false;
        function onFollowSpeedChange(val) {
            followSpeedEditing = true;
            const rpm = parseInt(val, 10);
            const pwm = Math.round((rpm / 75.0) * 255);
            document.getElementById('followSpeedLabel').textContent =
                rpm + ' RPM (' + pwm + ' PWM)';
            fetch('/api/follow_speed', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({rpm: rpm})
            })
            .then(r => r.json())
            .then(data => {
                if (data.follow_rpm !== undefined) {
                    document.getElementById('followSpeedLabel').textContent =
                        data.follow_rpm + ' RPM (' + data.follow_pwm + ' PWM)';
                }
                setTimeout(() => { followSpeedEditing = false; }, 600);
            })
            .catch(() => { followSpeedEditing = false; });
        }

        // Poll status every 500ms
        function updateStatus() {
            fetch('/api/status')
                .then(r => r.json())
                .then(data => {
                    // Sync lock state
                    if (data.web_locked !== undefined && data.web_locked !== webModeLocked) {
                        webModeLocked = !!data.web_locked;
                        updateLockButtonUI();
                        updateRemoteUI();
                    }

                    // Mode
                    const modeEl = document.getElementById('modeDisplay');
                    const modeNames = {0: 'RC', 1: 'FOLLOW', 2: 'FAILSAFE'};
                    const modeClasses = {0: 'mode-rc', 1: 'mode-follow', 2: 'mode-failsafe'};
                    if (data.web_override || data.web_locked) {
                        const label = data.web_action ? data.web_action.toUpperCase() : 'STANDBY (RC/FOLLOW OFF)';
                        modeEl.textContent = 'WEB (' + label + ')';
                        modeEl.className = 'stat-value mode-web';
                    } else {
                        modeEl.textContent = modeNames[data.mode] || 'UNKNOWN';
                        modeEl.className = 'stat-value ' + (modeClasses[data.mode] || '');
                    }

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
                        document.getElementById('position').textContent =
                            '(' + data.detection.cx + ', ' + data.detection.cy + ')';
                        document.getElementById('distance').textContent =
                            (data.detection.area_ratio * 100).toFixed(1) + '% frame';
                    } else {
                        document.getElementById('targetStatus').textContent = '🔴 No target';
                        document.getElementById('targetStatus').style.color = '#ff4444';
                        document.getElementById('position').textContent = '—';
                        document.getElementById('distance').textContent = '—';
                    }

                    // Owner Shirt Lock
                    if (data.owner_lock && data.owner_lock.locked && data.owner_lock.rgb) {
                        const rgb = data.owner_lock.rgb;
                        const swatch = '<span style="display:inline-block;width:12px;height:12px;border-radius:3px;border:1px solid #fff;vertical-align:middle;margin-right:5px;background:rgb(' + rgb[0] + ',' + rgb[1] + ',' + rgb[2] + ');"></span>';
                        document.getElementById('ownerLockStatus').innerHTML =
                            swatch + data.owner_lock.match_score + '% match';
                    } else {
                        document.getElementById('ownerLockStatus').textContent = '🔓 Waiting for person';
                    }

                    // Sync Follow Mode Speed slider if not actively dragging
                    if (!followSpeedEditing && data.follow_rpm !== undefined) {
                        document.getElementById('followSpeedSlider').value = data.follow_rpm;
                        document.getElementById('followSpeedLabel').textContent =
                            data.follow_rpm + ' RPM (' + data.follow_pwm + ' PWM)';
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
      - Web Remote Control override (Forward, Reverse, 360 Right, 360 Left, Universal Speed)
      - Dedicated Web Mode Lock (shuts off RC and Follow modes completely)
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

        # Web manual control state
        self.web_mode_locked = False
        self.web_override_until = 0.0
        self.web_action = None
        self.web_left_speed = 0
        self.web_right_speed = 0

        # Install custom log handler to capture logs
        self._setup_log_capture()

        # Register routes
        self._register_routes()

        # Current command values (updated by rover controller)
        self.current_speed = 0
        self.current_steer = 0

    def is_web_override_active(self):
        """Return True if a web manual control command is currently active."""
        return time.time() < self.web_override_until

    def is_web_mode_active(self):
        """Return True if Web Mode is locked ON or a web movement is currently active."""
        return self.web_mode_locked or (time.time() < self.web_override_until)

    def get_web_motor_targets(self):
        """Return (left_speed, right_speed) for the 20Hz RPi web mode loop."""
        if time.time() < self.web_override_until:
            return self.web_left_speed, self.web_right_speed
        self.web_action = None
        self.web_left_speed = 0
        self.web_right_speed = 0
        self.current_speed = 0
        self.current_steer = 0
        return 0, 0

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

        @self.app.route('/api/follow_action', methods=['POST'])
        def api_follow_action():
            data = request.get_json(silent=True) or {}
            action = data.get('action', '')
            if not self.rover.no_camera and self.rover.detector:
                if action == 'lock_owner':
                    self.rover.detector.lock_owner_from_current()
                elif action == 'reset_owner':
                    self.rover.detector.reset_owner_lock()
            return jsonify({'ok': True})

        @self.app.route('/api/follow_speed', methods=['POST'])
        def api_follow_speed():
            data = request.get_json(silent=True) or {}
            rpm = int(data.get('rpm', 20))
            set_rpm, set_pwm = self.rover.follower.set_follow_rpm(rpm)
            return jsonify({
                'ok': True,
                'follow_rpm': set_rpm,
                'follow_pwm': set_pwm
            })

        @self.app.route('/api/web_lock', methods=['POST'])
        def api_web_lock():
            data = request.get_json(silent=True) or {}
            locked = bool(data.get('locked', False))
            self.web_mode_locked = locked
            self.web_override_until = 0.0
            self.web_action = None
            self.web_left_speed = 0
            self.web_right_speed = 0
            self.current_speed = 0
            self.current_steer = 0
            self.rover.serial.send_web_lock(locked)
            if locked:
                logger.info("🔒 Web Mode LOCKED — RC and Follow modes completely shut off")
            else:
                logger.info("🔓 Web Mode UNLOCKED — Returned control to RC / Follow mode")
            return jsonify({
                'ok': True,
                'web_locked': self.web_mode_locked
            })

        @self.app.route('/api/web_control', methods=['POST'])
        def api_web_control():
            data = request.get_json(silent=True) or {}
            action = data.get('action', 'stop')
            speed_pct = max(0, min(100, int(data.get('speed_pct', 60))))
            pwm = int(round((speed_pct / 100.0) * 200))

            if action == 'forward':
                left_spd, right_spd = pwm, pwm
            elif action == 'reverse':
                left_spd, right_spd = -pwm, -pwm
            elif action == 'rotate_right':
                # 360 Rotate Right: Left motors forward (+), Right motors backward (-)
                left_spd, right_spd = pwm, -pwm
            elif action == 'rotate_left':
                # 360 Rotate Left: Right motors forward (+), Left motors backward (-)
                left_spd, right_spd = -pwm, pwm
            else:
                left_spd, right_spd = 0, 0
                action = 'stop'

            if action == 'stop' or pwm == 0:
                self.web_override_until = 0.0
                self.web_action = None
                self.web_left_speed = 0
                self.web_right_speed = 0
                self.current_speed = 0
                self.current_steer = 0
                self.rover.serial.send_web_drive(0, 0)
            else:
                # Auto-enable Web Mode lock when driving from Web so RC/Follow never mix
                if not self.web_mode_locked:
                    self.web_mode_locked = True
                    self.rover.serial.send_web_lock(True)
                    logger.info("🔒 Web Mode auto-locked on movement — RC & Follow shut off")
                self.web_override_until = time.time() + 1.5
                self.web_action = action
                self.web_left_speed = left_spd
                self.web_right_speed = right_spd
                self.current_speed = pwm if action != 'reverse' else -pwm
                self.rover.serial.send_web_drive(left_spd, right_spd)

            return jsonify({
                'ok': True,
                'action': action,
                'web_locked': self.web_mode_locked,
                'left_speed': left_spd,
                'right_speed': right_spd
            })

        @self.app.route('/api/status')
        def api_status():
            # Get detection info
            detection = None
            owner_lock = None
            det_age = float('inf')
            if not self.rover.no_camera and self.rover.detector.is_running():
                det_raw, det_age = self.rover.detector.get_detection()
                owner_lock = self.rover.detector.get_owner_status()
                if det_raw and det_age < 2.0:
                    detection = {
                        'cx': int(det_raw['cx']),
                        'cy': int(det_raw['cy']),
                        'w': int(det_raw['w']),
                        'h': int(det_raw['h']),
                        'area_ratio': round(float(det_raw['area_ratio']), 4),
                        'confidence': round(float(det_raw['confidence']), 3)
                    }

            follow_rpm, follow_pwm = self.rover.follower.get_follow_speed_setting()

            return jsonify({
                'mode': self.rover.serial.get_mode(),
                'web_locked': self.web_mode_locked,
                'web_override': self.is_web_override_active(),
                'web_action': self.web_action if self.is_web_override_active() else None,
                'speed': self.current_speed,
                'steer': self.current_steer,
                'follow_rpm': follow_rpm,
                'follow_pwm': follow_pwm,
                'arduino_connected': self.rover.serial.is_connected(),
                'rc_channels': self.rover.serial.get_rc_channels(),
                'detection': detection,
                'owner_lock': owner_lock,
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

                # Encode as JPEG (55% quality = 2x smaller over Wi-Fi, zero visible loss)
                ret, buffer = cv2.imencode('.jpg', frame,
                                           [cv2.IMWRITE_JPEG_QUALITY, 55])
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

            time.sleep(0.04)  # ~25 FPS low-latency stream rate

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
