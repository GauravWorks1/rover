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
# Follow-Me Controller (Constant Maintained Speed — Default 20 RPM)
# ==============================================================================

# Frame center reference (derived from actual camera resolution)
FRAME_CENTER_X  = CAMERA_WIDTH  // 2   # 320
FRAME_CENTER_Y  = CAMERA_HEIGHT // 2   # 240

# Target: person bounding box should occupy this fraction of the frame
# when the rover is at the ideal following distance
TARGET_AREA_RATIO = 0.17      # ~17% of frame = safe stopping distance (~1.6m away)
TOO_CLOSE_STOP_RATIO = 0.25   # Safety #1: Instant hard brake if person exceeds 25% of frame (<1.0m)
ALLOW_FOLLOW_REVERSE = False  # Safety #2: Never auto-reverse blindly in Follow Mode

# PID gains for STEERING (horizontal centering, error is -100 to +100 %)
KP_STEER = 1.05
KI_STEER = 0.02
KD_STEER = 0.15

# Dead zones (ignore small errors to prevent jitter)
STEER_DEADZONE_PX   = 25     # pixels from center X (for 640px width)
AREA_DEADZONE_RATIO = 0.012  # area ratio tolerance

# Constant Follow Mode Speed (24V 250W Yalu Geared Motor: ~75 RPM rated at 255 PWM)
MOTOR_MAX_RPM           = 75     # Rated max wheel RPM at 100% duty (255 PWM)
DEFAULT_FOLLOW_RPM      = 20     # Default constant Follow Mode speed = 20 RPM (~68 PWM)
MIN_FOLLOW_RPM          = 5      # Minimum selectable Follow Mode RPM on Web UI
MAX_FOLLOW_RPM          = 60     # Maximum selectable Follow Mode RPM on Web UI

# Legacy / safety limits
MIN_FOLLOW_SPEED    = 60
MAX_SPEED           = 204    # Absolute hard ceiling (60 RPM)
MAX_STEER           = 100    # Max steering angle (-100 to 100)

# Smoothing: max change per cycle
SPEED_RAMP_RATE     = 15     # smooth ramp up to the constant 20 RPM target
STEER_RAMP_RATE     = 35     # responsive steering for linear actuators

# ==============================================================================
# Timeouts & Safety
# ==============================================================================
LOST_TARGET_TIMEOUT     = 1.0     # seconds without detection -> stop quickly for safety
CONTROL_LOOP_RATE       = 15      # Hz, main control loop target rate
