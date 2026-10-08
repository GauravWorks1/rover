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

from config import CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS

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
        self.MAX_TRACK_FRAMES = 45   # Re-anchor every ~1.5s so bounding box size (distance) updates accurately!

        # Smooth position filters
        self._smooth_cx = None
        self._smooth_cy = None

        cv2.setNumThreads(4)

    def _create_tracker(self):
        """Handle different OpenCV version tracker APIs safely. Use MOSSE for max speed."""
        try:
            return cv2.TrackerMOSSE_create()
        except AttributeError:
            try:
                return cv2.legacy.TrackerMOSSE_create()
            except AttributeError:
                # Fallback to KCF if MOSSE is missing in this OpenCV version
                try:
                    return cv2.TrackerKCF_create()
                except AttributeError:
                    return cv2.legacy.TrackerKCF_create()

    def start(self):
        logger.info("Loading Hybrid Detect+Track System...")
        
        # Load Face/Upperbody Detector
        cascade_path = os.path.join(cv2.data.haarcascades, 'haarcascade_frontalface_default.xml')
        self.cascade = cv2.CascadeClassifier(cascade_path)
        
        if self.cascade.empty():
            raise RuntimeError("Failed to load Haar Cascade XML!")

        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
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
                cv2.putText(annotated, "TRACKING (MOSSE)", (x, max(15, y - 8)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                
                self.frames_since_detect += 1
                
                # Periodically re-anchor tracker to avoid slow drift
                if self.frames_since_detect > self.MAX_TRACK_FRAMES:
                    self.is_tracking = False
            else:
                # Tracker lost target
                self.is_tracking = False

        # ==========================================
        # MODE 2: DETECTING (Find person initially)
        # ==========================================
        if not self.is_tracking:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            boxes = self.cascade.detectMultiScale(gray, scaleFactor=1.15, minNeighbors=4, minSize=(35, 35))
            
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

                # Expand face downwards to capture upper body/torso
                pad_x = int(box_w * 0.25)
                pad_top = int(box_h * 0.1)
                pad_bot = int(box_h * 1.4)
                
                tx = max(0, x - pad_x)
                ty = max(0, y - pad_top)
                tw = min(w - tx, box_w + 2 * pad_x)
                th = min(h - ty, box_h + pad_top + pad_bot)
                track_box = (tx, ty, tw, th)

                best_detection = {
                    'cx': tx + (tw // 2),
                    'cy': ty + (th // 2),
                    'w': tw, 'h': th,
                    'x1': tx, 'y1': ty,
                    'x2': tx + tw, 'y2': ty + th,
                    'area': tw * th,
                    'area_ratio': float(tw * th) / float(w * h),
                    'confidence': 1.0
                }
                
                # Draw red box to show a fresh detection
                cv2.rectangle(annotated, (tx, ty), (tx + tw, ty + th), (0, 0, 255), 2)
                cv2.putText(annotated, "DETECTED (Haar)", (tx, max(15, ty - 8)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # Initialize fast tracker with expanded upper-body box
                self.tracker = self._create_tracker()
                self.tracker.init(frame, track_box)
                self.is_tracking = True
                self.frames_since_detect = 0

        # Smooth coordinates to eliminate jitter
        if best_detection:
            raw_cx = best_detection['cx']
            raw_cy = best_detection['cy']
            if self._smooth_cx is None:
                self._smooth_cx = raw_cx
                self._smooth_cy = raw_cy
            else:
                self._smooth_cx = int(0.70 * self._smooth_cx + 0.30 * raw_cx)
                self._smooth_cy = int(0.70 * self._smooth_cy + 0.30 * raw_cy)

            best_detection['cx'] = self._smooth_cx
            best_detection['cy'] = self._smooth_cy
            cv2.circle(annotated, (best_detection['cx'], best_detection['cy']), 5, (0, 0, 255), -1)
        else:
            self._smooth_cx = None
            self._smooth_cy = None
            
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
