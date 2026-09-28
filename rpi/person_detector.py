"""
Person Detection using Google MediaPipe (BlazePose).

Extremely fast on ARM CPUs. Tracks human torso landmarks 
to perfectly center the rover on a person.
"""

import cv2
import time
import threading
import logging
import mediapipe as mp
import mediapipe.python.solutions.pose as mp_pose
import mediapipe.python.solutions.drawing_utils as mp_drawing

from config import CAMERA_INDEX, CAMERA_FPS

logger = logging.getLogger(__name__)

class PersonDetector:
    def __init__(self):
        self.cap = None
        self._running = False
        self._thread = None

        self._lock = threading.Lock()
        self._detection = None
        self._frame = None
        self._annotated_frame = None
        self._detection_time = 0
        self._fps = 0.0

        # Initialize MediaPipe Pose explicitly
        self.mp_pose = mp_pose
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
            model_complexity=0  # 0 = Lightest/Fastest model
        )
        self.mp_drawing = mp_drawing

    def start(self):
        logger.info("Loading MediaPipe Lightweight Pose Tracker...")
        
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        
        # Lower resolution for massive speed boost
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {CAMERA_INDEX}")

        self._running = True
        self._thread = threading.Thread(target=self._detect_loop, daemon=True)
        self._thread.start()
        logger.info("MediaPipe tracking thread started")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        if self.cap:
            self.cap.release()
        self.pose.close()

    def _detect_loop(self):
        frame_count = 0
        fps_start_time = time.time()

        while self._running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            # Run MediaPipe tracking
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
        
        # MediaPipe needs RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(rgb_frame)

        best_detection = None
        annotated = frame.copy()

        if results.pose_landmarks:
            # We track the torso (Midpoint between shoulders and hips)
            landmarks = results.pose_landmarks.landmark
            
            # Get key points (normalized 0.0 to 1.0)
            l_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
            r_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
            l_hip = landmarks[self.mp_pose.PoseLandmark.LEFT_HIP]
            r_hip = landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP]

            # Only track if we can see the upper body
            if l_shoulder.visibility > 0.5 or r_shoulder.visibility > 0.5:
                
                # Calculate bounding box from shoulders to hips
                x_coords = [l_shoulder.x, r_shoulder.x, l_hip.x, r_hip.x]
                y_coords = [l_shoulder.y, r_shoulder.y, l_hip.y, r_hip.y]
                
                x_min = max(0, min(x_coords))
                x_max = min(1, max(x_coords))
                y_min = max(0, min(y_coords))
                y_max = min(1, max(y_coords))

                box_w = (x_max - x_min) * w
                box_h = (y_max - y_min) * h
                area = box_w * box_h

                cx = int(((x_min + x_max) / 2) * w)
                cy = int(((y_min + y_max) / 2) * h)

                best_detection = {
                    'cx': cx,
                    'cy': cy,
                    'w': int(box_w),
                    'h': int(box_h),
                    'x1': int(x_min * w),
                    'y1': int(y_min * h),
                    'x2': int(x_max * w),
                    'y2': int(y_max * h),
                    'area': int(area),
                    'area_ratio': float(area) / float(w * h),
                    'confidence': float((l_shoulder.visibility + r_shoulder.visibility) / 2)
                }

                # Draw tracking visuals
                self.mp_drawing.draw_landmarks(
                    annotated, 
                    results.pose_landmarks, 
                    self.mp_pose.POSE_CONNECTIONS
                )
                cv2.rectangle(annotated, (best_detection['x1'], best_detection['y1']), 
                             (best_detection['x2'], best_detection['y2']), (0, 255, 0), 2)
                cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)

        cv2.putText(annotated, f"MediaPipe FPS: {self._fps:.1f}", (10, 30),
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

