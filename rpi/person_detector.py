"""
Hybrid Detection + Tracking for Raspberry Pi 4.

Uses a Haar Cascade to initially find the person (Detector), 
then switches to an ultra-fast OpenCV KCF Tracker to follow 
them at 30 FPS.
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
        self.tracker = None
        
        self._running = False
        self._thread = None
        self._lock = threading.Lock()
        self._detection = None
        self._frame = None
        self._annotated_frame = None
        self._detection_time = 0
        self._fps = 0.0
        
        # Tracking state
        self.is_tracking = False
        self.frames_since_detect = 0
        self.MAX_TRACK_FRAMES = 60  # Re-detect every 60 frames (~2 seconds) to prevent drift

        cv2.setNumThreads(4)

    def _create_tracker(self):
        """Handle different OpenCV version tracker APIs safely."""
        try:
            return cv2.TrackerKCF_create()
        except AttributeError:
            # Fallback for OpenCV > 4.5.1 where trackers moved to legacy
            return cv2.legacy.TrackerKCF_create()

    def start(self):
        logger.info("Loading Hybrid Detect+Track System...")
        
        # Load Face/Upperbody Detector
        cascade_path = os.path.join(cv2.data.haarcascades, 'haarcascade_frontalface_default.xml')
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
        logger.info("Hybrid tracking thread started")

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
        annotated = frame.copy()
        best_detection = None

        # ==========================================
        # MODE 1: TRACKING (Fast, follows pixels)
        # ==========================================
        if self.is_tracking:
            success, box = self.tracker.update(frame)
            
            if success:
                # Tracker was successful
                x, y, box_w, box_h = [int(v) for v in box]
                area = box_w * box_h
                
                best_detection = {
                    'cx': x + (box_w // 2),
                    'cy': y + (box_h // 2),
                    'w': box_w,
                    'h': box_h,
                    'x1': x, 'y1': y,
                    'x2': x + box_w, 'y2': y + box_h,
                    'area': area,
                    'area_ratio': float(area) / float(w * h),
                    'confidence': 1.0
                }
                
                cv2.rectangle(annotated, (x, y), (x + box_w, y + box_h), (0, 255, 255), 2)
                cv2.putText(annotated, "TRACKING (KCF)", (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                
                self.frames_since_detect += 1
                
                # Force a re-detection occasionally to prevent the tracker from drifting
                if self.frames_since_detect > self.MAX_TRACK_FRAMES:
                    self.is_tracking = False
            else:
                # Tracker lost the target
                self.is_tracking = False

        # ==========================================
        # MODE 2: DETECTING (Find the person initially)
        # ==========================================
        if not self.is_tracking:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            boxes = self.cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5, minSize=(30, 30))
            
            best_area = 0
            best_box = None

            # Find largest face
            for (x, y, box_w, box_h) in boxes:
                area = box_w * box_h
                if area > best_area:
                    best_area = area
                    best_box = (x, y, box_w, box_h)
            
            if best_box is not None:
                x, y, box_w, box_h = best_box
                
                best_detection = {
                    'cx': x + (box_w // 2),
                    'cy': y + (box_h // 2),
                    'w': box_w, 'h': box_h,
                    'x1': x, 'y1': y,
                    'x2': x + box_w, 'y2': y + box_h,
                    'area': best_area,
                    'area_ratio': float(best_area) / float(w * h),
                    'confidence': 1.0
                }
                
                # Draw red box to show a fresh detection
                cv2.rectangle(annotated, (x, y), (x + box_w, y + box_h), (0, 0, 255), 2)
                cv2.putText(annotated, "DETECTED (Haar)", (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # Initialize the fast tracker with this bounding box
                self.tracker = self._create_tracker()
                self.tracker.init(frame, best_box)
                self.is_tracking = True
                self.frames_since_detect = 0

        # Draw crosshair and FPS
        if best_detection:
            cv2.circle(annotated, (best_detection['cx'], best_detection['cy']), 5, (0, 0, 255), -1)
            
        cv2.putText(annotated, f"Hybrid FPS: {self._fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
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
