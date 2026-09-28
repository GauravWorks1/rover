"""
Person Detection using OpenCV DNN with MobileNet SSD.

Runs inference in a separate thread for maximum frame rate.
Detects 'person' class and returns bounding box info.
"""

import cv2
import numpy as np
import threading
import time
import logging
import os

from config import (
    CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS,
    MODEL_PROTOTXT, MODEL_WEIGHTS,
    CONFIDENCE_THRESHOLD, PERSON_CLASS_ID,
    DETECT_INPUT_WIDTH, DETECT_INPUT_HEIGHT
)

logger = logging.getLogger(__name__)

# VOC class labels (MobileNet SSD trained on VOC)
VOC_CLASSES = [
    "background", "aeroplane", "bicycle", "bird", "boat",
    "bottle", "bus", "car", "cat", "chair", "cow",
    "diningtable", "dog", "horse", "motorbike", "person",
    "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]


class PersonDetector:
    """
    Threaded person detector using MobileNet SSD.

    Captures frames from the webcam in one thread,
    runs detection, and provides results via get_detection().
    """

    def __init__(self):
        self.net = None
        self.cap = None
        self._running = False
        self._thread = None

        # Latest detection result (thread-safe)
        self._lock = threading.Lock()
        self._detection = None       # (cx, cy, w, h, area, confidence) or None
        self._frame = None           # latest frame (for debug display)
        self._annotated_frame = None # frame with bounding box drawn
        self._detection_time = 0     # timestamp of last detection
        self._fps = 0.0

    def start(self):
        """Initialize camera and neural network, start detection thread."""
        # Resolve model paths relative to the project root
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_dir = os.path.dirname(script_dir)
        prototxt_path = os.path.join(project_dir, MODEL_PROTOTXT)
        weights_path = os.path.join(project_dir, MODEL_WEIGHTS)

        # Load neural network
        if not os.path.exists(prototxt_path):
            raise FileNotFoundError(
                f"Model prototxt not found: {prototxt_path}\n"
                f"Run: bash models/download_models.sh"
            )
        if not os.path.exists(weights_path):
            raise FileNotFoundError(
                f"Model weights not found: {weights_path}\n"
                f"Run: bash models/download_models.sh"
            )

        logger.info("Loading MobileNet SSD model...")
        self.net = cv2.dnn.readNetFromCaffe(prototxt_path, weights_path)

        # Use CPU backend (RPi4 doesn't have CUDA)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        logger.info("Model loaded successfully")

        # Open camera
        logger.info(f"Opening camera index {CAMERA_INDEX}...")
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera index {CAMERA_INDEX}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        logger.info(f"Camera opened: {actual_w}x{actual_h}")

        # Start detection thread
        self._running = True
        self._thread = threading.Thread(target=self._detect_loop, daemon=True)
        self._thread.start()
        logger.info("Person detection thread started")

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
                logger.warning("Failed to read frame from camera")
                time.sleep(0.01)
                continue

            # Run detection
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
                logger.debug(f"Detection FPS: {self._fps:.1f}")

    def _run_detection(self, frame):
        """
        Run MobileNet SSD on a single frame.

        Returns:
            detection: dict with keys {cx, cy, w, h, area, confidence} or None
            annotated: frame with bounding box drawn
        """
        h, w = frame.shape[:2]

        # Create input blob
        blob = cv2.dnn.blobFromImage(
            cv2.resize(frame, (DETECT_INPUT_WIDTH, DETECT_INPUT_HEIGHT)),
            scalefactor=0.007843,       # 1/127.5
            size=(DETECT_INPUT_WIDTH, DETECT_INPUT_HEIGHT),
            mean=(127.5, 127.5, 127.5),  # mean subtraction
            swapRB=False,
            crop=False
        )

        # Run inference
        self.net.setInput(blob)
        detections = self.net.forward()

        # Find the largest "person" detection (closest person)
        best_detection = None
        best_area = 0

        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            class_id = int(detections[0, 0, i, 1])

            if class_id != PERSON_CLASS_ID:
                continue
            if confidence < CONFIDENCE_THRESHOLD:
                continue

            # Bounding box (normalized coordinates)
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            x1, y1, x2, y2 = box.astype("int")

            # Clamp to frame boundaries
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(w, x2)
            y2 = min(h, y2)

            box_w = x2 - x1
            box_h = y2 - y1
            area = box_w * box_h

            if area > best_area:
                best_area = area
                best_detection = {
                    'cx': (x1 + x2) // 2,   # center x
                    'cy': (y1 + y2) // 2,   # center y
                    'w': box_w,
                    'h': box_h,
                    'x1': x1, 'y1': y1,
                    'x2': x2, 'y2': y2,
                    'area': area,
                    'area_ratio': area / (w * h),
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
            label = f"Person {d['confidence']:.0%} | Area: {d['area_ratio']:.1%}"
            cv2.putText(annotated, label, (d['x1'], d['y1'] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Draw frame center crosshair
        cv2.drawMarker(annotated, (w // 2, h // 2), (255, 0, 0),
                       cv2.MARKER_CROSS, 20, 1)

        # FPS overlay
        cv2.putText(annotated, f"FPS: {self._fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        return best_detection, annotated

    # =========================================================================
    # Public API
    # =========================================================================

    def get_detection(self):
        """
        Get the latest person detection result.

        Returns:
            detection: dict {cx, cy, w, h, area, area_ratio, confidence} or None
            age: seconds since this detection was made
        """
        with self._lock:
            if self._detection is None:
                return None, float('inf')
            age = time.time() - self._detection_time
            return dict(self._detection), age

    def get_frame(self):
        """Get the latest raw camera frame."""
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def get_annotated_frame(self):
        """Get the latest frame with detection overlay."""
        with self._lock:
            if self._annotated_frame is not None:
                return self._annotated_frame.copy()
            return None

    def get_fps(self):
        """Get current detection FPS."""
        return self._fps

    def is_running(self):
        """Check if detection is active."""
        return self._running and self._thread is not None and self._thread.is_alive()
