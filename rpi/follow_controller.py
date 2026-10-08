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
    TOO_CLOSE_STOP_RATIO,
    ALLOW_FOLLOW_REVERSE,
    KP_STEER, KI_STEER, KD_STEER,
    KP_SPEED, KI_SPEED, KD_SPEED,
    STEER_DEADZONE_PX, AREA_DEADZONE_RATIO,
    MIN_FOLLOW_SPEED, MAX_SPEED, MAX_STEER,
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

        # Speed PID: error is (target_area_ratio - current_area_ratio)
        # Safety #2: If ALLOW_FOLLOW_REVERSE is False, minimum speed is 0 (never reverse blindly)
        min_speed = -MAX_SPEED if ALLOW_FOLLOW_REVERSE else 0
        self.speed_pid = PIDController(
            KP_SPEED, KI_SPEED, KD_SPEED,
            min_speed, MAX_SPEED
        )

        # Smoothed outputs (for ramp limiting)
        self._current_speed = 0.0
        self._current_steer = 0.0

        # Target lost & safety tracking
        self._last_detection_time = None
        self._target_acquired = False
        self._too_close_active = False

    def compute(self, detection, detection_age):
        """
        Compute speed and steering commands from detection.

        Args:
            detection: dict from PersonDetector.get_detection() or None
            detection_age: seconds since detection was made

        Returns:
            (speed, steer):
            speed: 0 to MAX_SPEED (forward only when ALLOW_FOLLOW_REVERSE=False)
            steer: -100 to 100 (positive = turn right, negative = turn left)
        """
        # =====================================================================
        # Case 1: No detection or stale detection
        # =====================================================================
        if detection is None or detection_age > LOST_TARGET_TIMEOUT:
            if self._target_acquired:
                logger.info("Target lost — stopping")
                self._target_acquired = False
            self._too_close_active = False

            # Gradually slow down to stop
            self._current_speed = self._ramp(self._current_speed, 0, SPEED_RAMP_RATE)
            self._current_steer = self._ramp(self._current_steer, 0, STEER_RAMP_RATE)
            if not ALLOW_FOLLOW_REVERSE and self._current_speed < 0:
                self._current_speed = 0.0

            # Reset PIDs when target is lost
            if abs(self._current_speed) < 1 and abs(self._current_steer) < 1:
                self.steer_pid.reset()
                self.speed_pid.reset()

            return int(self._current_speed), int(self._current_steer)

        # =====================================================================
        # Case 2: Person detected — compute follow commands
        # =====================================================================
        if not self._target_acquired:
            logger.info(f"Target acquired! confidence={detection['confidence']:.0%}, area={detection['area_ratio']:.1%}")
            self._target_acquired = True

        self._last_detection_time = time.time()
        current_area = detection['area_ratio']

        # --- Steering: horizontal centering (ALWAYS active when person is detected!) ---
        # Offset in pixels from center (-320 to +320)
        # Positive offset = person is to the RIGHT
        offset_px = detection['cx'] - FRAME_CENTER_X

        # Apply pixel dead zone around center
        if abs(offset_px) < STEER_DEADZONE_PX:
            steer_error = 0.0
        else:
            # Normalize to percentage [-100.0, +100.0]
            steer_error = (offset_px / (CAMERA_WIDTH / 2.0)) * 100.0

        steer_target = self.steer_pid.compute(steer_error)
        self._current_steer = self._ramp(self._current_steer, steer_target,
                                          STEER_RAMP_RATE)

        # =====================================================================
        # SAFETY #1: Too-Close Emergency Stop (Instant Hard Brake on DRIVE motors)
        # Keep steering actuators active (self._current_steer) so wheels still
        # track left/right even when standing close to the camera!
        # =====================================================================
        if current_area >= TOO_CLOSE_STOP_RATIO:
            if not self._too_close_active:
                logger.warning(
                    f"⚠️ SAFETY STOP: Person too close ({current_area:.1%} >= {TOO_CLOSE_STOP_RATIO:.0%}) — Drive motors stopped, steering active!"
                )
                self._too_close_active = True
            self.speed_pid.reset()
            self._current_speed = 0.0
            return 0, int(self._current_steer)
        else:
            if self._too_close_active:
                logger.info("✅ Person at safe distance — resuming Follow Mode drive")
                self._too_close_active = False

        # --- Speed: distance control via bounding box area ---
        # Error = target_area_ratio - current_area_ratio
        # Positive error = person too far away -> move forward
        # Negative error = person closer than target -> stop (Safety #2: no reverse)
        area_error = TARGET_AREA_RATIO - current_area

        # Apply dead zone
        if abs(area_error) < AREA_DEADZONE_RATIO:
            area_error = 0.0

        # If person is at or closer than target distance and reverse is disabled,
        # stop immediately instead of slowly ramping down into the person
        if not ALLOW_FOLLOW_REVERSE and area_error <= 0.0:
            self.speed_pid.reset()
            speed_target = 0.0
            self._current_speed = 0.0
        else:
            raw_pid_speed = self.speed_pid.compute(area_error)

            # Reduce speed slightly when turning sharply (at most 25% reduction)
            turn_factor = 1.0 - 0.25 * abs(steer_target / MAX_STEER)
            raw_pid_speed *= turn_factor

            # Map positive PID output into [MIN_FOLLOW_SPEED .. MAX_SPEED]
            # so the 24V 250W geared motors always get enough starting torque to roll!
            if raw_pid_speed > 1.0:
                speed_target = max(float(MIN_FOLLOW_SPEED), min(float(MAX_SPEED), raw_pid_speed + MIN_FOLLOW_SPEED * 0.6))
            else:
                speed_target = 0.0

            # Jump-start from 0 to MIN_FOLLOW_SPEED so ramp doesn't stall below motor friction threshold
            if self._current_speed < MIN_FOLLOW_SPEED and speed_target >= MIN_FOLLOW_SPEED:
                self._current_speed = float(MIN_FOLLOW_SPEED)
            else:
                self._current_speed = self._ramp(self._current_speed, speed_target,
                                                  SPEED_RAMP_RATE)

        # Safety #2: Final hard clamp so Follow Mode never sends negative (reverse) speed
        if not ALLOW_FOLLOW_REVERSE and self._current_speed < 0:
            self._current_speed = 0.0

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
