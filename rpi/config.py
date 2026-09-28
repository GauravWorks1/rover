"""
Rover Configuration Constants
All tunable parameters in one place.
"""

# ==============================================================================
# Serial Communication (RPi <-> Arduino)
# ==============================================================================
SERIAL_PORT = '/dev/ttyACM0'      # Arduino Mega USB serial
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
# Camera
# ==============================================================================
CAMERA_INDEX    = 0               # USB webcam index
CAMERA_WIDTH    = 640
CAMERA_HEIGHT   = 480
CAMERA_FPS      = 30

# ==============================================================================
# Person Detection (MobileNet SSD)
# ==============================================================================
MODEL_PROTOTXT  = 'models/MobileNetSSD_deploy.prototxt'
MODEL_WEIGHTS   = 'models/MobileNetSSD_deploy.caffemodel'
CONFIDENCE_THRESHOLD = 0.50       # minimum detection confidence
PERSON_CLASS_ID = 15              # "person" in VOC class list

# Input blob size for the neural network
DETECT_INPUT_WIDTH  = 300
DETECT_INPUT_HEIGHT = 300

# ==============================================================================
# Follow-Me PID Controller
# ==============================================================================

# Frame center reference
FRAME_CENTER_X  = CAMERA_WIDTH  // 2   # 320
FRAME_CENTER_Y  = CAMERA_HEIGHT // 2   # 240
FRAME_AREA      = CAMERA_WIDTH * CAMERA_HEIGHT

# Target: person bounding box should occupy this fraction of the frame
# when the rover is at the ideal following distance
TARGET_AREA_RATIO = 0.12   # ~12% of frame = good following distance

# PID gains for STEERING (horizontal centering)
KP_STEER = 0.40
KI_STEER = 0.005
KD_STEER = 0.15

# PID gains for SPEED (distance keeping via bounding box area)
KP_SPEED = 200.0       # scaled to convert area-ratio error to speed %
KI_SPEED = 5.0
KD_SPEED = 30.0

# Dead zones (ignore small errors to prevent jitter)
STEER_DEADZONE_PX   = 30     # pixels from center X
AREA_DEADZONE_RATIO = 0.015  # area ratio tolerance

# Output limits
MAX_SPEED           = 180    # max PWM value (0-255), limit for safety
MAX_STEER           = 100    # max steering angle (-100 to 100)

# Smoothing: max change per cycle (prevents jerky movements)
SPEED_RAMP_RATE     = 10     # max speed change per control cycle
STEER_RAMP_RATE     = 8      # max steer change per control cycle

# ==============================================================================
# Timeouts & Safety
# ==============================================================================
LOST_TARGET_TIMEOUT     = 2.0     # seconds without detection -> stop
SEARCH_ROTATE_SPEED     = 0       # don't auto-rotate when target lost (safety)
CONTROL_LOOP_RATE       = 15      # Hz, main control loop target rate
SERIAL_WATCHDOG_TIMEOUT = 0.5     # seconds, Arduino stops if no command

# ==============================================================================
# RC Channel Mappings (for status display / logging)
# ==============================================================================
RC_CH_STEER     = 0   # CH1 - left/right
RC_CH_THROTTLE  = 1   # CH2 - forward/reverse
RC_CH_AUX       = 2   # CH3 - unused / speed trim
RC_CH_MODE      = 3   # CH5 - mode switch (mapped as index 3 in our status packet)

# RC mode switch threshold
RC_MODE_SWITCH_THRESHOLD = 1500   # CH5 > 1500 = Follow mode, < 1500 = RC mode
