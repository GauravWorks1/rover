"""
Hybrid 3-Thread Vision System for Raspberry Pi 4 (100% Offline):
  - Thread 1: Zero-Latency MJPG Camera Frame Grabber (30 FPS)
  - Thread 2: Real-Time MOSSE Tracker + Shirt-Color Owner Verification + Hand Gesture Control (25-30 FPS)
  - Thread 3: Asynchronous Background Person Detector:
      * Stage A: Haar Frontal Face (fast close/medium range)
      * Stage B: Haar Upper-Body (works from FRONT and BACK — head + shoulders)
      * Stage C: HOG Full-Body People Detector (works from FRONT, SIDE, and BACK)
"""

import cv2
import numpy as np
import time
import threading
import logging
import os

from config import CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS

logger = logging.getLogger(__name__)


class PersonDetector:
    def __init__(self):
        self.cap = None
        self.face_cascade = None
        self.upperbody_cascade = None
        self.hog = None
        self.tracker = None

        self._running = False
        self._capture_thread = None
        self._bg_detect_thread = None
        self._thread = None

        # Thread 1: Raw frame buffer
        self._raw_lock = threading.Lock()
        self._latest_raw_frame = None
        self._raw_frame_id = 0

        # Thread 3 -> Thread 2 handoff: Background detection candidate
        self._bg_lock = threading.Lock()
        self._bg_candidate = None       # dict with box, label, color_score, hist, bgr

        # Thread 2 output: Final detection + annotated frame
        self._lock = threading.Lock()
        self._detection = None
        self._frame = None
        self._annotated_frame = None
        self._detection_time = 0
        self._fps = 0.0

        # Tracking state
        self.is_tracking = False
        self.frames_since_detect = 0
        self.MAX_TRACK_FRAMES = 45      # Re-anchor every ~1.5s from background detector
        self._last_detect_label = "TRACKING"

        # Smooth position filters
        self._smooth_cx = None
        self._smooth_cy = None

        # =====================================================================
        # FEATURE #3: Shirt-Color "Owner Lock" State
        # =====================================================================
        self._owner_lock = threading.Lock()
        self.owner_color_enabled = True
        self.owner_hist = None          # 2D Hue-Saturation histogram of locked owner's torso
        self.owner_rgb = None           # (R, G, B) dominant shirt color for UI display
        self.owner_match_score = 0.0    # 0.0 to 1.0 similarity
        self.OWNER_MIN_MATCH = 0.38     # Reject other people with similarity < 38%

        # =====================================================================
        # FEATURE #1: Hand-Gesture Pause / Resume State
        # =====================================================================
        self.gesture_control_enabled = True
        self.gesture_paused = False     # True = Follow Mode paused (HOLD position)
        self._gesture_hold_start = None
        self._last_gesture_toggle = 0.0
        self._gesture_hand_box = None   # (x, y, w, h) when raised hand is detected
        self.GESTURE_HOLD_SEC = 0.75    # Hold raised hand for 0.75s to toggle Pause/Resume
        self.GESTURE_COOLDOWN_SEC = 2.5 # Minimum 2.5s between toggles

        cv2.setNumThreads(4)

    def _create_tracker(self):
        """Handle different OpenCV version tracker APIs safely. Use MOSSE for max speed."""
        for creator in (
            lambda: cv2.TrackerMOSSE_create(),
            lambda: cv2.legacy.TrackerMOSSE_create(),
            lambda: cv2.TrackerKCF_create(),
            lambda: cv2.legacy.TrackerKCF_create(),
        ):
            try:
                return creator()
            except AttributeError:
                continue
        return None

    def start(self):
        logger.info("Loading 3-Thread Hybrid Detect+Track + OwnerLock + Gesture System...")

        # 1. Load Face Cascade
        face_path = os.path.join(cv2.data.haarcascades, 'haarcascade_frontalface_default.xml')
        self.face_cascade = cv2.CascadeClassifier(face_path)
        if self.face_cascade.empty():
            raise RuntimeError("Failed to load Haar Frontal Face XML!")

        # 2. Load Upper-Body Cascade (detects head + shoulders from Front OR Back!)
        ub_path = os.path.join(cv2.data.haarcascades, 'haarcascade_upperbody.xml')
        if os.path.exists(ub_path):
            self.upperbody_cascade = cv2.CascadeClassifier(ub_path)
            if self.upperbody_cascade.empty():
                self.upperbody_cascade = None

        # 3. Initialize HOG Full-Body People Detector (detects standing person from Front, Side, or Back)
        try:
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        except Exception as e:
            logger.warning(f"HOG init warning: {e}")
            self.hog = None

        # Open V4L2 camera with hardware MJPG for zero USB lag
        try:
            self.cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(CAMERA_INDEX)
        except Exception:
            self.cap = cv2.VideoCapture(CAMERA_INDEX)

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {CAMERA_INDEX}")

        self._running = True

        # Thread 1: Zero-latency USB frame grabber
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        # Thread 3: Asynchronous background detector (Face + UpperBody + HOG FullBody)
        self._bg_detect_thread = threading.Thread(target=self._background_detect_loop, daemon=True)
        self._bg_detect_thread.start()

        # Thread 2: Real-time tracker + Gesture + Owner Color verification
        self._thread = threading.Thread(target=self._detect_loop, daemon=True)
        self._thread.start()

        logger.info("3-Thread Vision System started (Front/Back Body + Owner Color Lock + Hand Gestures)")

    def stop(self):
        self._running = False
        for t in (self._thread, self._bg_detect_thread, self._capture_thread):
            if t:
                t.join(timeout=2.0)
        if self.cap:
            self.cap.release()

    # =========================================================================
    # FEATURE #3: Shirt-Color "Owner Lock" Helpers
    # =========================================================================

    def _extract_torso_hist(self, frame, box):
        """
        Extract 2D Hue-Saturation histogram and dominant RGB color from the torso
        region of a person bounding box. Takes < 0.4ms!
        """
        h, w = frame.shape[:2]
        x, y, bw, bh = [int(v) for v in box]

        # Crop center torso patch (middle 50% width, 35%..80% height of person box)
        tx1 = max(0, x + int(bw * 0.25))
        tx2 = min(w, x + int(bw * 0.75))
        ty1 = max(0, y + int(bh * 0.35))
        ty2 = min(h, y + int(bh * 0.85))

        if tx2 - tx1 < 10 or ty2 - ty1 < 10:
            return None, None

        torso = frame[ty1:ty2, tx1:tx2]
        hsv = cv2.cvtColor(torso, cv2.COLOR_BGR2HSV)

        # Mask out very dark/bright glare pixels so lighting changes don't affect shirt color
        mask = cv2.inRange(hsv, np.array([0, 25, 30]), np.array([180, 255, 245]))
        hist = cv2.calcHist([hsv], [0, 1], mask, [18, 16], [0, 180, 0, 256])
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)

        # Compute average RGB of torso for Web UI badge
        mean_bgr = cv2.mean(torso, mask=mask)[:3]
        if sum(mean_bgr) < 10:
            mean_bgr = cv2.mean(torso)[:3]
        rgb = (int(mean_bgr[2]), int(mean_bgr[1]), int(mean_bgr[0]))
        return hist, rgb

    def _compare_owner_hist(self, candidate_hist):
        """Return similarity score (0.0 to 1.0) against locked owner's shirt histogram."""
        if candidate_hist is None:
            return 0.0
        with self._owner_lock:
            if not self.owner_color_enabled or self.owner_hist is None:
                return 1.0
            score = cv2.compareHist(self.owner_hist, candidate_hist, cv2.HISTCMP_CORREL)
            return float(max(0.0, min(1.0, (score + 1.0) / 2.0 if score < 0 else score)))

    def lock_owner_from_current(self):
        """Lock (or re-lock) owner shirt color from the currently tracked person."""
        with self._lock:
            frame = self._frame.copy() if self._frame is not None else None
            det = dict(self._detection) if self._detection is not None else None
        if frame is not None and det is not None:
            box = (det['x1'], det['y1'], det['w'], det['h'])
            hist, rgb = self._extract_torso_hist(frame, box)
            if hist is not None:
                with self._owner_lock:
                    self.owner_hist = hist
                    self.owner_rgb = rgb
                    self.owner_match_score = 1.0
                logger.info(f"👕 Owner shirt color locked! RGB={rgb}")
                return True
        return False

    def reset_owner_lock(self):
        """Clear the locked shirt color so the next detected person becomes the new owner."""
        with self._owner_lock:
            self.owner_hist = None
            self.owner_rgb = None
            self.owner_match_score = 0.0
        self.is_tracking = False
        logger.info("🔓 Owner shirt color lock reset — will lock onto next person")

    def get_owner_status(self):
        """Return dict of owner lock status for Web UI."""
        with self._owner_lock:
            return {
                'enabled': self.owner_color_enabled,
                'locked': self.owner_hist is not None,
                'rgb': self.owner_rgb,
                'match_score': round(self.owner_match_score * 100, 1)
            }

    # =========================================================================
    # FEATURE #1: Hand Gesture Detection (Raised Palm Beside Head/Shoulder)
    # =========================================================================

    def set_gesture_paused(self, paused: bool):
        """Manually set or toggle Gesture Pause state (from Web UI or gesture)."""
        self.gesture_paused = bool(paused)
        self._last_gesture_toggle = time.time()
        state_str = "PAUSED (HOLD)" if self.gesture_paused else "RESUMED (FOLLOWING)"
        logger.info(f"✋ Follow State -> {state_str}")

    def _check_hand_gesture(self, frame, box):
        """
        Check the upper-left and upper-right shoulder/head zones next to the person
        for a raised open hand (YCrCb skin-tone blob with palm aspect/solidity).
        Holding a raised hand for 0.75s toggles Follow Pause/Resume!
        """
        if not self.gesture_control_enabled:
            self._gesture_hand_box = None
            return

        h, w = frame.shape[:2]
        x, y, bw, bh = [int(v) for v in box]

        # Only check gestures when person is reasonably visible
        if bw < 50 or bh < 70:
            self._gesture_hand_box = None
            self._gesture_hold_start = None
            return

        # Define raised-hand search regions on left and right sides of upper body/head
        roi_w = max(45, int(bw * 0.55))
        roi_h = max(60, int(bh * 0.55))
        ry1 = max(0, y - int(bh * 0.10))
        ry2 = min(h, ry1 + roi_h)

        left_rx1 = max(0, x - int(roi_w * 0.75))
        left_rx2 = min(w, left_rx1 + roi_w)

        right_rx1 = max(0, x + bw - int(roi_w * 0.25))
        right_rx2 = min(w, right_rx1 + roi_w)

        found_hand = None
        for (rx1, rx2) in ((left_rx1, left_rx2), (right_rx1, right_rx2)):
            if rx2 - rx1 < 25 or ry2 - ry1 < 25:
                continue
            roi = frame[ry1:ry2, rx1:rx2]
            ycrcb = cv2.cvtColor(roi, cv2.COLOR_BGR2YCrCb)
            skin_mask = cv2.inRange(ycrcb, np.array([0, 133, 77]), np.array([255, 173, 127]))
            skin_mask = cv2.GaussianBlur(skin_mask, (5, 5), 0)

            contours, _ = cv2.findContours(skin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            roi_area = float((rx2 - rx1) * (ry2 - ry1))

            for cnt in contours:
                area = cv2.contourArea(cnt)
                # Raised palm occupies ~14% to 65% of the shoulder ROI
                if roi_area * 0.14 <= area <= roi_area * 0.68:
                    hx, hy, hw, hh = cv2.boundingRect(cnt)
                    aspect = float(hw) / float(max(1, hh))
                    hull = cv2.convexHull(cnt)
                    hull_area = max(1.0, cv2.contourArea(hull))
                    solidity = area / hull_area
                    # Open palm/hand has moderate solidity (0.52..0.92) and upright aspect
                    if 0.45 <= aspect <= 1.45 and 0.50 <= solidity <= 0.92:
                        found_hand = (rx1 + hx, ry1 + hy, hw, hh)
                        break
            if found_hand is not None:
                break

        now = time.time()
        if found_hand is not None:
            self._gesture_hand_box = found_hand
            if self._gesture_hold_start is None:
                self._gesture_hold_start = now
            elif (now - self._gesture_hold_start >= self.GESTURE_HOLD_SEC and
                  now - self._last_gesture_toggle >= self.GESTURE_COOLDOWN_SEC):
                self.gesture_paused = not self.gesture_paused
                self._last_gesture_toggle = now
                self._gesture_hold_start = None
                state_txt = "✋ PAUSED (HOLDING POSITION)" if self.gesture_paused else "▶️ RESUMED FOLLOWING"
                logger.info(f"Hand Gesture Triggered -> {state_txt}")
        else:
            self._gesture_hand_box = None
            self._gesture_hold_start = None

    # =========================================================================
    # THREAD 1: Zero-Latency USB Camera Frame Grabber
    # =========================================================================

    def _capture_loop(self):
        """Continuously drain USB webcam buffer so frames are never delayed."""
        while self._running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.005)
                continue
            with self._raw_lock:
                self._latest_raw_frame = frame
                self._raw_frame_id += 1

    # =========================================================================
    # THREAD 3: Asynchronous Background Person Detector (Front, Side, & Back!)
    # =========================================================================

    def _background_detect_loop(self):
        """
        Runs on a separate CPU thread so the live video stream NEVER lags!
        Detects people via:
          1. Haar Frontal Face (close/mid range)
          2. Haar Upper-Body (works from FRONT or BACK — head & shoulders!)
          3. HOG Full-Body Detector (works from FRONT, SIDE, or BACK!)
        Scores candidates using Owner Shirt-Color Lock.
        """
        last_bg_frame_id = -1

        while self._running:
            with self._raw_lock:
                if self._latest_raw_frame is None or self._raw_frame_id == last_bg_frame_id:
                    frame = None
                else:
                    frame = self._latest_raw_frame.copy()
                    last_bg_frame_id = self._raw_frame_id

            if frame is None:
                time.sleep(0.03)
                continue

            h, w = frame.shape[:2]
            small = cv2.resize(frame, (w // 2, h // 2), interpolation=cv2.INTER_LINEAR)
            small_gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

            candidates = []  # list of (tx, ty, tw, th, label)

            # --- Stage A: Frontal Face (expanded to upper body) ---
            faces = self.face_cascade.detectMultiScale(
                small_gray, scaleFactor=1.2, minNeighbors=4, minSize=(18, 18),
                flags=cv2.CASCADE_SCALE_IMAGE
            )
            for (sx, sy, sw, sh) in faces:
                x, y, bw, bh = sx * 2, sy * 2, sw * 2, sh * 2
                pad_x = int(bw * 0.25)
                pad_top = int(bh * 0.10)
                pad_bot = int(bh * 1.40)
                tx = max(0, x - pad_x)
                ty = max(0, y - pad_top)
                tw = min(w - tx, bw + 2 * pad_x)
                th = min(h - ty, bh + pad_top + pad_bot)
                candidates.append((tx, ty, tw, th, "FACE+BODY"))

            # --- Stage B: Upper-Body (Head + Shoulders — works from BACK or FRONT!) ---
            if not candidates and self.upperbody_cascade is not None:
                ubodies = self.upperbody_cascade.detectMultiScale(
                    small_gray, scaleFactor=1.18, minNeighbors=3, minSize=(32, 32),
                    flags=cv2.CASCADE_SCALE_IMAGE
                )
                for (sx, sy, sw, sh) in ubodies:
                    tx, ty, tw, th = sx * 2, sy * 2, sw * 2, int(sh * 2.2)
                    th = min(h - ty, th)
                    candidates.append((tx, ty, tw, th, "UPPER-BODY (FRONT/BACK)"))

            # --- Stage C: HOG Full-Body Person Detector (works from BACK, SIDE, or FRONT!) ---
            if not candidates and self.hog is not None:
                rects, weights = self.hog.detectMultiScale(
                    small_gray, winStride=(8, 8), padding=(8, 8), scale=1.08
                )
                for i, (sx, sy, sw, sh) in enumerate(rects):
                    if weights[i] >= 0.35:
                        tx = max(0, sx * 2 + int(sw * 0.15))
                        ty = max(0, sy * 2 + int(sh * 0.10))
                        tw = min(w - tx, int(sw * 1.7))
                        th = min(h - ty, int(sh * 1.7))
                        candidates.append((tx, ty, tw, th, "FULL-BODY (BACK/FRONT)"))

            # Pick the best candidate that matches Owner Shirt Color (if locked)
            best_cand = None
            best_score = -1.0

            for (tx, ty, tw, th, label) in candidates:
                hist, rgb = self._extract_torso_hist(frame, (tx, ty, tw, th))
                color_match = self._compare_owner_hist(hist)

                with self._owner_lock:
                    has_owner = self.owner_color_enabled and (self.owner_hist is not None)

                # If Owner Color Lock is active, reject strangers whose shirt doesn't match
                if has_owner and color_match < self.OWNER_MIN_MATCH:
                    continue

                area_ratio = float(tw * th) / float(w * h)
                combined_score = (color_match * 2.0) + area_ratio
                if combined_score > best_score:
                    best_score = combined_score
                    best_cand = {
                        'box': (tx, ty, tw, th),
                        'label': label,
                        'hist': hist,
                        'rgb': rgb,
                        'color_match': color_match,
                        'time': time.time()
                    }

            if best_cand is not None:
                # Auto-lock onto the first detected person's shirt if not locked yet
                with self._owner_lock:
                    if self.owner_color_enabled and self.owner_hist is None and best_cand['hist'] is not None:
                        self.owner_hist = best_cand['hist']
                        self.owner_rgb = best_cand['rgb']
                        self.owner_match_score = 1.0
                        best_cand['color_match'] = 1.0
                        logger.info(f"👕 Auto-locked Owner Shirt Color: RGB={self.owner_rgb}")

                with self._bg_lock:
                    self._bg_candidate = best_cand

            # Run background detector ~4-5 times per second so CPU stays cool
            time.sleep(0.18)

    # =========================================================================
    # THREAD 2: Fast Real-Time Tracker + HUD Overlay (25-30 FPS)
    # =========================================================================

    def _detect_loop(self):
        frame_count = 0
        fps_start_time = time.time()
        last_processed_id = -1

        while self._running:
            with self._raw_lock:
                if self._latest_raw_frame is None or self._raw_frame_id == last_processed_id:
                    frame = None
                else:
                    frame = self._latest_raw_frame
                    last_processed_id = self._raw_frame_id

            if frame is None:
                time.sleep(0.005)
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

        # Check if Thread 3 (Background Detector) found a fresh/re-anchored person box
        fresh_bg = None
        with self._bg_lock:
            if self._bg_candidate is not None and (time.time() - self._bg_candidate['time'] < 0.6):
                fresh_bg = self._bg_candidate
                self._bg_candidate = None

        if fresh_bg is not None and (not self.is_tracking or self.frames_since_detect >= self.MAX_TRACK_FRAMES):
            tx, ty, tw, th = fresh_bg['box']
            self.tracker = self._create_tracker()
            if self.tracker is not None:
                self.tracker.init(frame, (tx, ty, tw, th))
                self.is_tracking = True
                self.frames_since_detect = 0
                self._last_detect_label = fresh_bg['label']
            with self._owner_lock:
                self.owner_match_score = fresh_bg['color_match']

        # ==========================================
        # FAST TRACKING (25-30 FPS)
        # ==========================================
        if self.is_tracking and self.tracker is not None:
            success, box = self.tracker.update(frame)
            if success:
                x, y, box_w, box_h = [int(v) for v in box]
                x = max(0, min(w - 10, x))
                y = max(0, min(h - 10, y))
                box_w = max(10, min(w - x, box_w))
                box_h = max(10, min(h - y, box_h))

                # Verify Owner Shirt-Color match periodically (every 5 frames = ~0.15ms avg)
                if self.frames_since_detect % 5 == 0:
                    hist, _ = self._extract_torso_hist(frame, (x, y, box_w, box_h))
                    match = self._compare_owner_hist(hist)
                    with self._owner_lock:
                        self.owner_match_score = match
                        has_owner = self.owner_color_enabled and (self.owner_hist is not None)
                    # If tracker drifted onto background or wrong person, drop tracker
                    if has_owner and match < (self.OWNER_MIN_MATCH * 0.85):
                        self.is_tracking = False

                if self.is_tracking:
                    # Check Hand Gesture in shoulder/head zone
                    self._check_hand_gesture(frame, (x, y, box_w, box_h))

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
                        'confidence': max(0.5, self.owner_match_score),
                        'gesture_paused': self.gesture_paused
                    }

                    box_color = (0, 180, 255) if self.gesture_paused else (0, 255, 255)
                    cv2.rectangle(annotated, (x, y), (x + box_w, y + box_h), box_color, 2)
                    cv2.putText(
                        annotated,
                        f"{self._last_detect_label} ({int(self.owner_match_score * 100)}% match)",
                        (x, max(18, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2
                    )
                    self.frames_since_detect += 1
            else:
                self.is_tracking = False

        # Draw Raised Hand Gesture Box if detected
        if self._gesture_hand_box is not None:
            hx, hy, hw, hh = self._gesture_hand_box
            cv2.rectangle(annotated, (hx, hy), (hx + hw, hy + hh), (255, 0, 255), 2)
            cv2.putText(annotated, "HAND GESTURE", (hx, max(15, hy - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 2)

        # Smooth coordinates to eliminate jitter
        if best_detection:
            raw_cx = best_detection['cx']
            raw_cy = best_detection['cy']
            if self._smooth_cx is None:
                self._smooth_cx = raw_cx
                self._smooth_cy = raw_cy
            else:
                self._smooth_cx = int(0.65 * self._smooth_cx + 0.35 * raw_cx)
                self._smooth_cy = int(0.65 * self._smooth_cy + 0.35 * raw_cy)

            best_detection['cx'] = self._smooth_cx
            best_detection['cy'] = self._smooth_cy
            cv2.circle(annotated, (best_detection['cx'], best_detection['cy']), 5, (0, 0, 255), -1)
        else:
            self._smooth_cx = None
            self._smooth_cy = None

        # HUD Overlays (FPS, Owner Color Swatch, Gesture State)
        cv2.putText(annotated, f"FPS: {self._fps:.1f}", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

        if self.gesture_paused:
            cv2.putText(annotated, "GESTURE: PAUSED (HOLD)", (10, 56),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2)

        with self._owner_lock:
            if self.owner_rgb is not None:
                r, g, b = self.owner_rgb
                cv2.rectangle(annotated, (10, h - 35), (35, h - 10), (b, g, r), -1)
                cv2.rectangle(annotated, (10, h - 35), (35, h - 10), (255, 255, 255), 1)
                cv2.putText(annotated, f"OWNER LOCK ({int(self.owner_match_score * 100)}%)",
                            (42, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

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
