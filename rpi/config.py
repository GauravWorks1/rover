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
TARGET_AREA_RATIO = 0.12   # ~12% of frame = good following distance

# PID gains for STEERING (horizontal centering, error is -100 to +100 %)
KP_STEER = 0.80
KI_STEER = 0.01
KD_STEER = 0.10

# PID gains for SPEED (distance keeping via bounding box area)
KP_SPEED = 800.0       # scaled to convert area-ratio error to speed PWM
KI_SPEED = 5.0
KD_SPEED = 30.0

# Dead zones (ignore small errors to prevent jitter)
STEER_DEADZONE_PX   = 30     # pixels from center X (for 640px width)
AREA_DEADZONE_RATIO = 0.015  # area ratio tolerance

# Output limits
MAX_SPEED           = 200    # max PWM value (0-255), matches Arduino MAX_MOTOR_PWM
MAX_STEER           = 100    # max steering angle (-100 to 100)

# Smoothing: max change per cycle (prevents jerky movements)
SPEED_RAMP_RATE     = 20     # max speed change per control cycle
STEER_RAMP_RATE     = 25     # fast steering response for linear actuators

# ==============================================================================
# Timeouts & Safety
# ==============================================================================
LOST_TARGET_TIMEOUT     = 2.0     # seconds without detection -> stop
CONTROL_LOOP_RATE       = 15      # Hz, main control loop target rate
