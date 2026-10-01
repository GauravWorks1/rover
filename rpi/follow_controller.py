"""
PID-based Follow Controller.

Takes person detection data (bounding box position and size) and
outputs motor commands (speed + steering) to follow the person.

Steering: keeps the person centered horizontally in the frame.
Speed:    keeps the person at a target distance (based on bounding box area).
"""

import time
import logging

from config import (
    FRAME_CENTER_X,
    TARGET_AREA_RATIO,
    KP_STEER, KI_STEER, KD_STEER,
    KP_SPEED, KI_SPEED, KD_SPEED,
    STEER_DEADZONE_PX, AREA_DEADZONE_RATIO,
    MAX_SPEED, MAX_STEER,
    SPEED_RAMP_RATE, STEER_RAMP_RATE,
    LOST_TARGET_TIMEOUT, CAMERA_WIDTH
)

logger = logging.getLogger(__name__)


class PIDController:
    """Simple PID controller with anti-windup."""

    def __init__(self, kp, ki, kd, output_min, output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, error):
        """
        Compute PID output for the given error.

        Args:
            error: current error value

        Returns:
            PID output, clamped to [output_min, output_max]
        """
        now = time.time()

        if self.prev_time is None:
            dt = 0.066  # assume ~15 Hz on first call
        else:
            dt = now - self.prev_time
            dt = max(dt, 0.001)  # prevent division by zero

        self.prev_time = now

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup
        self.integral += error * dt
        # Clamp integral to prevent windup
        max_integral = (self.output_max - self.output_min) / (self.ki + 1e-6)
        self.integral = max(-max_integral, min(max_integral, self.integral))
        i_term = self.ki * self.integral

        # Derivative
        d_term = self.kd * (error - self.prev_error) / dt
        self.prev_error = error

        # Sum and clamp
        output = p_term + i_term + d_term
        output = max(self.output_min, min(self.output_max, output))

        return output

    def reset(self):
        """Reset PID state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None


class FollowController:
    """
    Converts person detection results into rover motor commands.

    Uses two PID controllers:
      1. Steering PID: keeps person centered (horizontal error -> steering angle)
      2. Speed PID: maintains following distance (area error -> speed)
    """

    def __init__(self):
        # Steering PID: error is horizontal offset in pixels
        # Output: steering value (-MAX_STEER to +MAX_STEER)
        # Positive error (person is right of center) -> positive steer (turn right)
        self.steer_pid = PIDController(
            KP_STEER, KI_STEER, KD_STEER,
            -MAX_STEER, MAX_STEER
        )

        # Speed PID: error is (current_area_ratio - target_area_ratio)
        # Positive error (person too close) -> negative speed (slow down / reverse)
        # Negative error (person too far) -> positive speed (speed up)
        self.speed_pid = PIDController(
            KP_SPEED, KI_SPEED, KD_SPEED,
            -MAX_SPEED, MAX_SPEED
        )

        # Smoothed outputs (for ramp limiting)
        self._current_speed = 0.0
        self._current_steer = 0.0

        # Target lost tracking
        self._last_detection_time = None
        self._target_acquired = False

    def compute(self, detection, detection_age):
        """
        Compute speed and steering commands from detection.

        Args:
            detection: dict from PersonDetector.get_detection() or None
            detection_age: seconds since detection was made

        Returns:
            (speed, steer): both in range [-100, 100] approx
            speed: positive = forward, negative = reverse
            steer: positive = turn right, negative = turn left
        """
        # =====================================================================
        # Case 1: No detection or stale detection
        # =====================================================================
        if detection is None or detection_age > LOST_TARGET_TIMEOUT:
            if self._target_acquired:
                logger.info("Target lost — stopping")
                self._target_acquired = False

            # Gradually slow down to stop
            self._current_speed = self._ramp(self._current_speed, 0, SPEED_RAMP_RATE)
            self._current_steer = self._ramp(self._current_steer, 0, STEER_RAMP_RATE)

            # Reset PIDs when target is lost
            if abs(self._current_speed) < 1 and abs(self._current_steer) < 1:
                self.steer_pid.reset()
                self.speed_pid.reset()

            return int(self._current_speed), int(self._current_steer)

        # =====================================================================
        # Case 2: Person detected — compute follow commands
        # =====================================================================
        if not self._target_acquired:
            logger.info(f"Target acquired! confidence={detection['confidence']:.0%}")
            self._target_acquired = True

        self._last_detection_time = time.time()

        # --- Steering: horizontal centering ---
        # Offset in pixels from center (-160 to +160)
        # Positive offset = person is to the RIGHT
        offset_px = detection['cx'] - FRAME_CENTER_X

        # Apply pixel dead zone around center
        if abs(offset_px) < STEER_DEADZONE_PX:
            steer_error = 0.0
        else:
            # Normalize to percentage [-100.0, +100.0]
            steer_error = (offset_px / (CAMERA_WIDTH / 2.0)) * 100.0

        steer_target = self.steer_pid.compute(steer_error)

        # --- Speed: distance control via bounding box area ---
        # Error = target_area_ratio - current_area_ratio
        # Positive error = person too far away -> move forward
        # Negative error = person too close -> slow down / reverse
        area_error = TARGET_AREA_RATIO - detection['area_ratio']

        # Apply dead zone
        if abs(area_error) < AREA_DEADZONE_RATIO:
            area_error = 0.0

        speed_target = self.speed_pid.compute(area_error)

        # Reduce speed when turning sharply (safety)
        turn_factor = 1.0 - 0.5 * abs(steer_target / MAX_STEER)
        speed_target *= turn_factor

        # --- Apply ramp limiting (smooth acceleration) ---
        self._current_speed = self._ramp(self._current_speed, speed_target,
                                          SPEED_RAMP_RATE)
        self._current_steer = self._ramp(self._current_steer, steer_target,
                                          STEER_RAMP_RATE)

        return int(self._current_speed), int(self._current_steer)

    def reset(self):
        """Reset controller state (call when switching modes)."""
        self.steer_pid.reset()
        self.speed_pid.reset()
        self._current_speed = 0.0
        self._current_steer = 0.0
        self._target_acquired = False
        logger.info("Follow controller reset")

    @property
    def target_acquired(self):
        """Whether the controller currently has a target."""
        return self._target_acquired

    @staticmethod
    def _ramp(current, target, max_rate):
        """
        Ramp-limit a value: move current toward target by at most max_rate per step.
        """
        diff = target - current
        if abs(diff) <= max_rate:
            return target
        return current + max_rate * (1 if diff > 0 else -1)
