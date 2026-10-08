"""
Rover Configuration Constants
All tunable parameters in one place.
"""

# ==============================================================================
# Serial Communication (RPi <-> Arduino)
# ==============================================================================
SERIAL_PORT = '/dev/ttyUSB0'      # Arduino Mega USB serial
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.05             # 50ms read timeout

# Protocol bytes
START_BYTE      = 0xAA
END_BYTE        = 0x55
STATUS_START    = 0xBB

# Command types (RPi -> Arduino)
CMD_DRIVE       = 0x01
CMD_STOP        = 0x02
CMD_QUERY       = 0x03
CMD_WEB_DRIVE   = 0x04   # Direct Left/Right tank motor override from Web UI
CMD_WEB_LOCK    = 0x05   # Dedicated Web Mode Lock (1=Shut off RC & Follow, 0=Release)

# Packet sizes
CMD_PACKET_SIZE     = 8   # RPi -> Arduino
STATUS_PACKET_SIZE  = 12  # Arduino -> RPi

# ==============================================================================
# Operating Modes
# ==============================================================================
MODE_RC         = 0
MODE_FOLLOW     = 1
MODE_FAILSAFE   = 2

# ==============================================================================
# Camera (MUST match person_detector.py capture resolution!)
# ==============================================================================
CAMERA_INDEX    = 0               # USB webcam index
CAMERA_WIDTH    = 640             # 640x480 resolution
CAMERA_HEIGHT   = 480
CAMERA_FPS      = 30

# ==============================================================================
# Follow-Me PID Controller
# ==============================================================================

# Frame center reference (derived from actual camera resolution)
FRAME_CENTER_X  = CAMERA_WIDTH  // 2   # 320
FRAME_CENTER_Y  = CAMERA_HEIGHT // 2   # 240

# Target: person bounding box should occupy this fraction of the frame
# when the rover is at the ideal following distance
# Note: Haar face box expanded to upper body is typically 4% (far) to 16% (ideal) to 28%+ (very close)
TARGET_AREA_RATIO = 0.16    # ~16% of frame = ideal stopping distance (~1.5m away)
TOO_CLOSE_STOP_RATIO = 0.28 # Safety #1: Instant hard brake if person exceeds 28% of frame (<0.8m)
ALLOW_FOLLOW_REVERSE = False # Safety #2: Never auto-reverse blindly in Follow Mode

# PID gains for STEERING (horizontal centering, error is -100 to +100 %)
KP_STEER = 1.10
KI_STEER = 0.02
KD_STEER = 0.12

# PID gains for SPEED (distance keeping via bounding box area)
KP_SPEED = 1400.0      # scaled to convert area-ratio error to strong motor PWM
KI_SPEED = 15.0
KD_SPEED = 25.0

# Dead zones (ignore small errors to prevent jitter)
STEER_DEADZONE_PX   = 25     # pixels from center X (for 640px width)
AREA_DEADZONE_RATIO = 0.010  # area ratio tolerance

# Output limits
MIN_FOLLOW_SPEED    = 85     # Minimum PWM needed to overcome heavy rover static friction
MAX_SPEED           = 180    # Max Follow PWM (0-255)
MAX_STEER           = 100    # Max steering angle (-100 to 100)

# Smoothing: max change per cycle (prevents jerky movements)
SPEED_RAMP_RATE     = 25     # max speed change per control cycle
STEER_RAMP_RATE     = 35     # fast steering response for linear actuators

# ==============================================================================
# Timeouts & Safety
# ==============================================================================
LOST_TARGET_TIMEOUT     = 2.0     # seconds without detection -> stop
CONTROL_LOOP_RATE       = 15      # Hz, main control loop target rate
