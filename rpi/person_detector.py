"""
Person Detection using Google MediaPipe (BlazePose).

Extremely fast on ARM CPUs. Tracks human torso landmarks 
to perfectly center the rover on a person.
"""

"""
Person Detection using OpenCV Haar Cascades (Ultra Lightweight).

Runs extremely fast on Raspberry Pi CPUs without needing 
any heavy neural networks or external pip packages.
"""

import cv2
import time
import threading
import logging
import os

from config import CAMERA_INDEX, CAMERA_FPS

logger = logging.getLogger(__name__)

class PersonDetector:
    def __init__(self):
        self.cap = None
        self.cascade = None
        self._running = False
        self._thread = None

        self._lock = threading.Lock()
        self._detection = None
        self._frame = None
        self._annotated_frame = None
        self._detection_time = 0
        self._fps = 0.0

        # Maximize CPU usage
        cv2.setNumThreads(4)

    def start(self):
        logger.info("Loading Ultra-Lightweight Haar Cascade...")
        
        # Load the built-in OpenCV Full Body detector
        cascade_path = os.path.join(cv2.data.haarcascades, 'haarcascade_fullbody.xml')
        self.cascade = cv2.CascadeClassifier(cascade_path)
        
        if self.cascade.empty():
            raise RuntimeError("Failed to load Haar Cascade XML!")

        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        
        # Low resolution for massive speed boost
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {CAMERA_INDEX}")

        self._running = True
        self._thread = threading.Thread(target=self._detect_loop, daemon=True)
        self._thread.start()
        logger.info("Haar Cascade tracking thread started")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        if self.cap:
            self.cap.release()

    def _detect_loop(self):
        frame_count = 0
        fps_start_time = time.time()

        while self._running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            detection, annotated = self._run_detection(frame)

            with self._lock:
                self._frame = frame
                self._annotated_frame = annotated
                self._detection = detection
                if detection is not None:
                    self._detection_time = time.time()

            # FPS calc
            frame_count += 1
            elapsed = time.time() - fps_start_time
            if elapsed >= 1.0:
                self._fps = frame_count / elapsed
                frame_count = 0
                fps_start_time = time.time()

    def _run_detection(self, frame):
        h, w = frame.shape[:2]
        
        # Haar Cascades require grayscale images (makes it much faster!)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect bodies
        boxes = self.cascade.detectMultiScale(
            gray, 
            scaleFactor=1.1, 
            minNeighbors=3, 
            minSize=(30, 80)  # Don't look for tiny specs
        )

        best_detection = None
        best_area = 0
        annotated = frame.copy()

        for (x, y, box_w, box_h) in boxes:
            area = box_w * box_h
            if area > best_area:
                best_area = area
                
                cx = x + (box_w // 2)
                cy = y + (box_h // 2)

                best_detection = {
                    'cx': int(cx),
                    'cy': int(cy),
                    'w': int(box_w),
                    'h': int(box_h),
                    'x1': int(x),
                    'y1': int(y),
                    'x2': int(x + box_w),
                    'y2': int(y + box_h),
                    'area': int(area),
                    'area_ratio': float(area) / float(w * h),
                    'confidence': 1.0  # Cascades don't output confidence natively
                }

        if best_detection is not None:
            d = best_detection
            # Draw tracking visuals
            cv2.rectangle(annotated, (d['x1'], d['y1']), (d['x2'], d['y2']), (0, 255, 0), 2)
            cv2.circle(annotated, (d['cx'], d['cy']), 5, (0, 0, 255), -1)

        cv2.putText(annotated, f"Cascade FPS: {self._fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.drawMarker(annotated, (w // 2, h // 2), (255, 0, 0), cv2.MARKER_CROSS, 20, 1)

        return best_detection, annotated

    def get_detection(self):
        with self._lock:
            if self._detection is None:
                return None, float('inf')
            return dict(self._detection), time.time() - self._detection_time

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

