"""
Main Rover Controller — orchestrates all subsystems.

Manages mode switching between RC and Follow-Me,
coordinates person detection, PID following, and serial communication.
Includes a web-based live monitoring dashboard.
"""

import time
import signal
import sys
import logging
import argparse

from config import (
    MODE_RC, MODE_FOLLOW, MODE_FAILSAFE,
    CONTROL_LOOP_RATE, LOST_TARGET_TIMEOUT
)
from web_monitor import WebMonitor
from serial_comm import SerialComm
from person_detector import PersonDetector
from follow_controller import FollowController

logger = logging.getLogger(__name__)


class RoverController:
    """
    High-level rover controller.

    Reads the current mode from Arduino (set by FlySky CH5 switch):
      - MODE_RC:     Arduino handles motors directly from FlySky. RPi is idle.
      - MODE_FOLLOW: RPi runs person detection + PID and sends drive commands.
      - MODE_FAILSAFE: Everything stopped. Arduino handles this internally.
    """

    def __init__(self, serial_port=None, no_camera=False, web_port=5000):
        self.serial = SerialComm(port=serial_port) if serial_port else SerialComm()
        self.detector = PersonDetector()
        self.follower = FollowController()
        self.web_monitor = None
        self.web_port = web_port

        self.no_camera = no_camera  # for testing without webcam
        self._running = False
        self._current_mode = MODE_RC
        self._prev_mode = MODE_RC

        # Stats
        self._loop_count = 0
        self._start_time = 0

    def start(self):
        """Initialize all subsystems and start the main loop."""
        logger.info("=" * 60)
        logger.info("  ROVER CONTROLLER STARTING")
        logger.info("=" * 60)

        # Connect to Arduino
        logger.info("Connecting to Arduino...")
        if not self.serial.connect():
            logger.error("Failed to connect to Arduino. Check USB connection.")
            logger.error("Is the Arduino plugged in? Is the serial port correct?")
            return False

        # Start person detector
        if not self.no_camera:
            logger.info("Starting person detector...")
            try:
                self.detector.start()
            except Exception as e:
                logger.error(f"Failed to start person detector: {e}")
                logger.warning("Running without camera — RC mode only")
                self.no_camera = True

        # Start web monitor dashboard
        logger.info("Starting web monitor...")
        self.web_monitor = WebMonitor(self, port=self.web_port)
        self.web_monitor.start()

        # Register signal handlers for clean shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        logger.info("All systems initialized. Starting main loop...")
        logger.info("Mode is controlled by FlySky SwB switch:")
        logger.info("  SwB DOWN = Follow-Me Mode (camera follows person)")
        logger.info("  SwB UP   = RC Mode (FlySky stick driving)")
        logger.info("")

        self._running = True
        self._start_time = time.time()

        try:
            self._main_loop()
        except Exception as e:
            logger.error(f"Main loop crashed: {e}", exc_info=True)
        finally:
            self.stop()

        return True

    def stop(self):
        """Gracefully shut down all subsystems."""
        logger.info("Shutting down rover controller...")
        self._running = False

        # Send stop command
        try:
            self.serial.send_stop()
            time.sleep(0.1)
            self.serial.send_stop()  # send twice for safety
        except Exception:
            pass

        # Stop subsystems
        if not self.no_camera:
            self.detector.stop()
        self.serial.disconnect()

        elapsed = time.time() - self._start_time if self._start_time else 0
        logger.info(f"Shutdown complete. Ran for {elapsed:.1f}s, "
                     f"{self._loop_count} control loops")

    def _signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully."""
        logger.info("\nShutdown signal received")
        self._running = False

    # =========================================================================
    # Main Control Loop
    # =========================================================================

    def _main_loop(self):
        """
        Main control loop. Runs at CONTROL_LOOP_RATE Hz.

        Each iteration:
          1. Read current mode from Arduino
          2. If Follow mode: run detection -> PID -> send commands
          3. If RC mode: just monitor (Arduino handles motors)
          4. Send heartbeat to keep Arduino watchdog happy
        """
        loop_period = 1.0 / CONTROL_LOOP_RATE

        while self._running:
            loop_start = time.time()

            # Check serial connection
            if not self.serial.is_connected():
                logger.error("Lost Arduino connection! Attempting reconnect...")
                time.sleep(1.0)
                self.serial.connect()
                continue

            # Read current mode from Arduino
            self._current_mode = self.serial.get_mode()

            # Handle mode transitions
            if self._current_mode != self._prev_mode:
                self._on_mode_change(self._prev_mode, self._current_mode)
                self._prev_mode = self._current_mode

            # Execute mode-specific logic (RC Mode or Follow Mode via FlySky SwB)
            if self._current_mode == MODE_FOLLOW and not self.no_camera:
                self._follow_mode_tick()
            elif self._current_mode == MODE_RC:
                self._rc_mode_tick()
            elif self._current_mode == MODE_FAILSAFE:
                self._failsafe_tick()
            else:
                # Follow mode requested but no camera
                if self._current_mode == MODE_FOLLOW and self.no_camera:
                    self.serial.send_stop()

            self._loop_count += 1

            # Log status periodically
            if self._loop_count % (CONTROL_LOOP_RATE * 5) == 0:
                self._log_status()

            # Maintain loop rate
            elapsed = time.time() - loop_start
            sleep_time = loop_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _on_mode_change(self, old_mode, new_mode):
        """Handle mode transitions."""
        mode_names = {MODE_RC: "RC", MODE_FOLLOW: "FOLLOW", MODE_FAILSAFE: "FAILSAFE"}
        old_name = mode_names.get(old_mode, f"UNKNOWN({old_mode})")
        new_name = mode_names.get(new_mode, f"UNKNOWN({new_mode})")
        logger.info(f"Mode change: {old_name} -> {new_name}")

        if new_mode == MODE_FOLLOW:
            # Reset PID controller for fresh start
            self.follower.reset()
        elif new_mode == MODE_RC:
            # Stop any autonomous movement
            self.serial.send_stop()
        elif new_mode == MODE_FAILSAFE:
            self.serial.send_stop()
            self.follower.reset()

    def _follow_mode_tick(self):
        """One iteration of follow-me mode."""
        # Get latest detection
        detection, age = self.detector.get_detection()

        # Compute motor commands via PID
        speed, steer = self.follower.compute(detection, age)

        # Update web monitor with current values
        if self.web_monitor:
            self.web_monitor.current_speed = speed
            self.web_monitor.current_steer = steer

        # Send to Arduino
        self.serial.send_drive(speed, steer)

    def _rc_mode_tick(self):
        """One iteration of RC mode — RPi just monitors."""
        # In RC mode, Arduino handles motors directly from FlySky
        # We just send periodic queries to stay connected (watchdog)
        # and monitor RC channel values for logging
        self.serial.send_query()

    def _failsafe_tick(self):
        """Failsafe mode — everything stopped."""
        self.serial.send_stop()

    def _log_status(self):
        """Log periodic status info."""
        mode_names = {MODE_RC: "RC", MODE_FOLLOW: "FOLLOW", MODE_FAILSAFE: "FAILSAFE"}
        mode_name = mode_names.get(self._current_mode, "UNKNOWN")
        channels = self.serial.get_rc_channels()

        status_parts = [f"Mode: {mode_name}"]
        status_parts.append(f"RC: [{', '.join(str(c) for c in channels)}]")

        if not self.no_camera:
            status_parts.append(f"Det FPS: {self.detector.get_fps():.1f}")
            if self.follower.target_acquired:
                detection, age = self.detector.get_detection()
                if detection:
                    status_parts.append(
                        f"Person: ({detection['cx']},{detection['cy']}) "
                        f"area={detection['area_ratio']:.1%}"
                    )

        logger.info(" | ".join(status_parts))


# =============================================================================
# Entry Point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="Rover Follow-Me Controller")
    parser.add_argument("--port", type=str, default=None,
                        help="Serial port for Arduino (default: /dev/ttyACM0)")
    parser.add_argument("--no-camera", action="store_true",
                        help="Run without camera (RC mode only)")
    parser.add_argument("--debug", action="store_true",
                        help="Enable debug logging")
    parser.add_argument("--log-file", type=str, default=None,
                        help="Log to file (in addition to console)")
    parser.add_argument("--web-port", type=int, default=5000,
                        help="Web monitor port (default: 5000)")
    args = parser.parse_args()

    # Configure logging
    log_level = logging.DEBUG if args.debug else logging.INFO
    log_format = "%(asctime)s [%(levelname)-5s] %(name)-20s: %(message)s"
    handlers = [logging.StreamHandler(sys.stdout)]
    if args.log_file:
        handlers.append(logging.FileHandler(args.log_file))

    logging.basicConfig(level=log_level, format=log_format, handlers=handlers)

    # Suppress noisy OpenCV logs
    logging.getLogger('cv2').setLevel(logging.WARNING)

    # Start rover
    rover = RoverController(
        serial_port=args.port,
        no_camera=args.no_camera,
        web_port=args.web_port
    )
    rover.start()


if __name__ == "__main__":
    main()
