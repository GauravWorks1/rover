"""
Follow Controller with Constant Maintained Speed (Default 20 RPM).

Takes person detection data (bounding box position and size) and
outputs motor commands (speed + steering) to follow the person safely:
  - Steering: PID keeps the person centered horizontally in the frame.
  - Speed:    Maintains a CONSTANT speed (20 RPM by default, customizable from Web UI
              for Follow Mode only) whenever the person is further than the target
              stopping distance, and stops immediately when at/closer than target distance.
"""

import time
import threading
import logging

from config import (
    FRAME_CENTER_X,
    TARGET_AREA_RATIO,
    TOO_CLOSE_STOP_RATIO,
    ALLOW_FOLLOW_REVERSE,
    KP_STEER, KI_STEER, KD_STEER,
    STEER_DEADZONE_PX, AREA_DEADZONE_RATIO,
    MOTOR_MAX_RPM, DEFAULT_FOLLOW_RPM, MIN_FOLLOW_RPM, MAX_FOLLOW_RPM,
    MAX_SPEED, MAX_STEER,
    SPEED_RAMP_RATE, STEER_RAMP_RATE,
    LOST_TARGET_TIMEOUT, CAMERA_WIDTH
)

logger = logging.getLogger(__name__)


class PIDController:
    """Simple PID controller with anti-windup (used for smooth horizontal steering)."""

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
        now = time.time()

        if self.prev_time is None:
            dt = 0.066  # assume ~15 Hz on first call
        else:
            dt = now - self.prev_time
            dt = max(dt, 0.001)

        self.prev_time = now

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup
        self.integral += error * dt
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
    Converts person detection results into safe, constant-speed rover motor commands.

      1. Steering PID: keeps person centered (horizontal error -> steering angle)
      2. Constant Follow Speed: drives at fixed RPM (20 RPM default, adjustable on Web UI
         for Follow Mode only) without speeding up as distance increases.
    """

    def __init__(self):
        self.steer_pid = PIDController(
            KP_STEER, KI_STEER, KD_STEER,
            -MAX_STEER, MAX_STEER
        )

        # Follow Mode Constant Speed State (Default: 20 RPM)
        self._speed_lock = threading.Lock()
        self._follow_rpm = int(DEFAULT_FOLLOW_RPM)
        self._follow_pwm = self._rpm_to_pwm(self._follow_rpm)

        # Smoothed outputs (for ramp limiting)
        self._current_speed = 0.0
        self._current_steer = 0.0

        # Target lost & safety tracking
        self._last_detection_time = None
        self._target_acquired = False
        self._too_close_active = False

    @staticmethod
    def _rpm_to_pwm(rpm):
        """Convert desired wheel RPM into 8-bit motor driver PWM (0-255)."""
        if rpm <= 0:
            return 0
        pwm = int(round((float(rpm) / float(MOTOR_MAX_RPM)) * 255.0))
        return max(0, min(int(MAX_SPEED), pwm))

    def set_follow_rpm(self, rpm):
        """Set custom constant Follow Mode speed in RPM (from Web UI)."""
        rpm_clamped = max(int(MIN_FOLLOW_RPM), min(int(MAX_FOLLOW_RPM), int(rpm)))
        pwm = self._rpm_to_pwm(rpm_clamped)
        with self._speed_lock:
            self._follow_rpm = rpm_clamped
            self._follow_pwm = pwm
        logger.info(f"⚙️ Follow Mode constant speed set to {rpm_clamped} RPM ({pwm} PWM)")
        return rpm_clamped, pwm

    def get_follow_speed_setting(self):
        """Return (follow_rpm, follow_pwm) for Web UI display."""
        with self._speed_lock:
            return self._follow_rpm, self._follow_pwm

    def compute(self, detection, detection_age):
        """
        Compute speed and steering commands from detection.

        Returns:
            (speed, steer):
            speed: 0 or constant follow_pwm (never increases above set Follow RPM!)
            steer: -100 to 100 (positive = turn right, negative = turn left)
        """
        # =====================================================================
        # Case 1: No detection or stale detection -> Stop quickly for safety
        # =====================================================================
        if detection is None or detection_age > LOST_TARGET_TIMEOUT:
            if self._target_acquired:
                logger.info("Target lost — stopping safely")
                self._target_acquired = False
            self._too_close_active = False

            # Stop drive motors promptly when target leaves view
            self._current_speed = 0.0
            self._current_steer = self._ramp(self._current_steer, 0, STEER_RAMP_RATE)

            if abs(self._current_steer) < 1:
                self.steer_pid.reset()

            return 0, int(self._current_steer)

        # =====================================================================
        # Case 2: Person detected — compute steering + constant speed
        # =====================================================================
        if not self._target_acquired:
            logger.info(f"Target acquired! confidence={detection['confidence']:.0%}, area={detection['area_ratio']:.1%}")
            self._target_acquired = True

        self._last_detection_time = time.time()
        current_area = detection['area_ratio']

        # --- Steering: horizontal centering (ALWAYS active when person is detected!) ---
        offset_px = detection['cx'] - FRAME_CENTER_X

        if abs(offset_px) < STEER_DEADZONE_PX:
            steer_error = 0.0
        else:
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
            self._current_speed = 0.0
            return 0, int(self._current_steer)
        else:
            if self._too_close_active:
                logger.info("✅ Person at safe distance — resuming Follow Mode drive")
                self._too_close_active = False

        # =====================================================================
        # Constant Maintained Follow Speed (Default 20 RPM — never increases!)
        # =====================================================================
        area_error = TARGET_AREA_RATIO - current_area

        # If person is at or closer than target stopping distance, stop immediately
        if area_error <= AREA_DEADZONE_RATIO:
            self._current_speed = 0.0
        else:
            # Person is further than stopping distance -> maintain constant configured RPM/PWM!
            with self._speed_lock:
                constant_pwm = float(self._follow_pwm)

            self._current_speed = constant_pwm

        # Safety #2: Final hard clamp so Follow Mode never sends negative (reverse) speed
        if not ALLOW_FOLLOW_REVERSE and self._current_speed < 0:
            self._current_speed = 0.0

        return int(self._current_speed), int(self._current_steer)

    def reset(self):
        """Reset controller state (call when switching modes)."""
        self.steer_pid.reset()
        self._current_speed = 0.0
        self._current_steer = 0.0
        self._target_acquired = False
        self._too_close_active = False
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
