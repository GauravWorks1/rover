"""
Person Detection using OpenCV HOG Descriptor (Extreme Lightweight).

Runs inference in a separate thread for maximum frame rate.
Requires NO external model files. Uses OpenCV built-in algorithms.
"""

import cv2
import numpy as np
import threading
import time
import logging

from config import (
    CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS,
    CONFIDENCE_THRESHOLD, DETECT_INPUT_WIDTH
)

logger = logging.getLogger(__name__)


class PersonDetector:
    """
    Threaded person detector using OpenCV HOG (Extremely Lightweight).
    """

    def __init__(self):
        self.cap = None
        self.hog = None
        self._running = False
        self._thread = None

        # Latest detection result (thread-safe)
        self._lock = threading.Lock()
        self._detection = None
        self._frame = None
        self._annotated_frame = None
        self._detection_time = 0
        self._fps = 0.0
        
        # Maximize OpenCV CPU threads
        cv2.setNumThreads(4)

    def start(self):
        """Initialize camera and HOG detector, start detection thread."""
        logger.info("Loading Lightweight HOG People Detector...")
        
        # Initialize the HOG descriptor/person detector
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        logger.info("HOG loaded successfully")

        # Open camera
        logger.info(f"Opening camera index {CAMERA_INDEX}...")
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        
        # Force lower resolution for speed
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera index {CAMERA_INDEX}")

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        logger.info(f"Camera opened: {actual_w}x{actual_h}")

        # Start detection thread
        self._running = True
        self._thread = threading.Thread(target=self._detect_loop, daemon=True)
        self._thread.start()
        logger.info("Lightweight person detection thread started")

    def stop(self):
        """Stop detection thread and release camera."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        if self.cap:
            self.cap.release()
        logger.info("Person detector stopped")

    def _detect_loop(self):
        """Main detection loop running in a background thread."""
        frame_count = 0
        fps_start_time = time.time()

        while self._running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            # Run HOG detection
            detection, annotated = self._run_detection(frame)

            # Update shared state
            with self._lock:
                self._frame = frame
                self._annotated_frame = annotated
                self._detection = detection
                if detection is not None:
                    self._detection_time = time.time()

            # FPS calculation
            frame_count += 1
            elapsed = time.time() - fps_start_time
            if elapsed >= 2.0:
                self._fps = frame_count / elapsed
                frame_count = 0
                fps_start_time = time.time()

    def _run_detection(self, frame):
        """Run HOG detection on a single frame."""
        h, w = frame.shape[:2]

        # Detect people in the image
        # winStride determines step size (smaller = more accurate but slower)
        # scale determines image pyramid scale (smaller = more accurate but slower)
        boxes, weights = self.hog.detectMultiScale(
            frame, 
            winStride=(8, 8), 
            padding=(4, 4), 
            scale=1.05
        )

        best_detection = None
        best_area = 0

        # Find the largest bounding box (closest person)
        for i, (x, y, box_w, box_h) in enumerate(boxes):
            confidence = weights[i]
            
            # HOG returns very different confidence numbers than DNN. 
            # Usually > 0.5 is okay, > 1.0 is very confident.
            if confidence < 0.3:
                continue

            area = box_w * box_h

            if area > best_area:
                best_area = area
                best_detection = {
                    'cx': int(x + (box_w / 2)),
                    'cy': int(y + (box_h / 2)),
                    'w': int(box_w),
                    'h': int(box_h),
                    'x1': int(x), 'y1': int(y),
                    'x2': int(x + box_w), 'y2': int(y + box_h),
                    'area': int(area),
                    'area_ratio': float(area) / float(w * h),
                    'confidence': float(confidence)
                }

        # Draw annotation on frame copy
        annotated = frame.copy()
        if best_detection is not None:
            d = best_detection
            # Green bounding box
            cv2.rectangle(annotated, (d['x1'], d['y1']), (d['x2'], d['y2']),
                          (0, 255, 0), 2)
            # Center crosshair
            cv2.circle(annotated, (d['cx'], d['cy']), 5, (0, 0, 255), -1)
            # Label
            cv2.putText(annotated, f"Person {d['confidence']:.2f}", (d['x1'], d['y1'] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Draw frame center crosshair
        cv2.drawMarker(annotated, (w // 2, h // 2), (255, 0, 0),
                       cv2.MARKER_CROSS, 20, 1)

        # FPS overlay
        cv2.putText(annotated, f"HOG FPS: {self._fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        return best_detection, annotated

    # =========================================================================
    # Public API
    # =========================================================================

    def get_detection(self):
        with self._lock:
            if self._detection is None:
                return None, float('inf')
            age = time.time() - self._detection_time
            return dict(self._detection), age

    def get_frame(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def get_annotated_frame(self):
        with self._lock:
            if self._annotated_frame is not None:
                return self._annotated_frame.copy()
            return None

    def get_fps(self):
        return self._fps

    def is_running(self):
        return self._running and self._thread is not None and self._thread.is_alive()

