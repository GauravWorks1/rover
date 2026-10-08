"""
Hybrid 3-Thread Vision System for Raspberry Pi 4 (100% Offline, Zero Lag):
  - Thread 1: Zero-Latency MJPG Camera Frame Grabber (30 FPS)
  - Thread 2: Fast Frontal-Face Detection + 30 FPS Tracker (MOSSE / Template fallback)
              + Shirt-Color Owner Verification + Calibrated Hand-Gesture Control
  - Thread 3: On-Demand Back / Full-Body Detector (Haar Upper-Body + Fast HOG)
              Only activates when frontal face is not visible (e.g. walking away with back turned),
              keeping RPi 4 CPU cool and 30 FPS video completely lag-free!
"""

import cv2
import numpy as np
import time
import threading
import logging
import os

from config import CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS

logger = logging.getLogger(__name__)


class _FastTemplateTracker:
    """
    Ultra-fast local template tracker fallback (takes < 1.5ms per frame on RPi 4).
    Guarantees 30 FPS tracking even on OpenCV builds without opencv-contrib MOSSE.
    """

    def __init__(self):
        self.template = None
        self.box = None

    def init(self, frame, box):
        h, w = frame.shape[:2]
        x, y, bw, bh = [int(v) for v in box]
        x = max(0, min(w - 10, x))
        y = max(0, min(h - 10, y))
        bw = max(10, min(w - x, bw))
        bh = max(10, min(h - y, bh))
        self.box = (x, y, bw, bh)
        gray = cv2.cvtColor(frame[y:y + bh, x:x + bw], cv2.COLOR_BGR2GRAY)
        self.template = cv2.resize(gray, (max(12, bw // 2), max(12, bh // 2)), interpolation=cv2.INTER_AREA)
        return True

    def update(self, frame):
        if self.template is None or self.box is None:
            return False, (0, 0, 0, 0)

        h, w = frame.shape[:2]
        x, y, bw, bh = self.box
        th, tw = self.template.shape[:2]

        # Search within a local window around previous position
        margin_x = max(40, int(bw * 0.6))
        margin_y = max(35, int(bh * 0.5))
        sx1 = max(0, x - margin_x)
        sy1 = max(0, y - margin_y)
        sx2 = min(w, x + bw + margin_x)
        sy2 = min(h, y + bh + margin_y)

        roi = frame[sy1:sy2, sx1:sx2]
        if roi.shape[0] <= bh or roi.shape[1] <= bw:
            return False, self.box

        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        small_roi = cv2.resize(
            roi_gray,
            (max(tw + 2, roi.shape[1] // 2), max(th + 2, roi.shape[0] // 2)),
            interpolation=cv2.INTER_AREA
        )

        res = cv2.matchTemplate(small_roi, self.template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(res)

        if max_val < 0.42:
            return False, self.box

        nx = sx1 + max_loc[0] * 2
        ny = sy1 + max_loc[1] * 2
        nx = max(0, min(w - bw, nx))
        ny = max(0, min(h - bh, ny))
        self.box = (nx, ny, bw, bh)

        # Gently adapt template (10% blend) to handle slow posture changes
        new_patch = cv2.cvtColor(frame[ny:ny + bh, nx:nx + bw], cv2.COLOR_BGR2GRAY)
        new_small = cv2.resize(new_patch, (tw, th), interpolation=cv2.INTER_AREA)
        self.template = cv2.addWeighted(self.template, 0.90, new_small, 0.10, 0)

        return True, self.box


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

        # Thread 3 -> Thread 2 handoff: Back/Full-Body candidate when face isn't visible
        self._bg_lock = threading.Lock()
        self._bg_candidate = None

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
        self.MAX_TRACK_FRAMES = 30      # Re-anchor every ~1.0s so tracker never drifts
        self._last_detect_label = "TRACKING"
        self._last_face_box = None      # (fx, fy, fw, fh) of most recent frontal face
        self._last_face_time = 0.0
        self._face_streak = 0           # Consecutive face detections before auto-locking shirt

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
        self.OWNER_MIN_MATCH = 0.30     # Minimum color correlation to accept back-view / multi-person match

        # =====================================================================
        # FEATURE #1: Hand-Gesture Pause / Resume State
        # =====================================================================
        self.gesture_control_enabled = True
        self.gesture_paused = False     # True = Follow Mode paused (HOLD position)
        self._gesture_hold_start = None
        self._gesture_progress = 0.0    # 0.0 to 1.0 visual hold progress bar
        self._last_gesture_toggle = 0.0
        self._gesture_hand_box = None   # (x, y, w, h) when raised hand is detected
        self.GESTURE_HOLD_SEC = 1.0     # Hold raised hand beside head for 1.0s to toggle Pause/Resume
        self.GESTURE_COOLDOWN_SEC = 3.0 # Minimum 3.0s between toggles

        # Limit OpenCV internal threads to 2 so Python threads + serial loop never starve
        cv2.setNumThreads(2)

    def _create_tracker(self):
        """Use OpenCV MOSSE tracker if available, or fast built-in template tracker."""
        for creator in (
            lambda: cv2.TrackerMOSSE_create(),
            lambda: cv2.legacy.TrackerMOSSE_create(),
            lambda: cv2.TrackerKCF_create(),
            lambda: cv2.legacy.TrackerKCF_create(),
        ):
            try:
                t = creator()
                if t is not None:
                    return t
            except Exception:
                continue
        return _FastTemplateTracker()

    def start(self):
        logger.info("Loading 3-Thread Hybrid Detect+Track + OwnerLock + Gesture System...")

        # 1. Load Frontal Face Cascade (primary fast detector)
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

        # 3. Initialize HOG Full-Body People Detector (detects person from Back, Side, or Front)
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

        # Thread 3: On-demand background back/full-body detector
        self._bg_detect_thread = threading.Thread(target=self._background_detect_loop, daemon=True)
        self._bg_detect_thread.start()

        # Thread 2: Real-time face detect + 30 FPS tracker + Gesture + Owner Color
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
        region of a person upper-body bounding box. Takes < 0.3ms!
        """
        h, w = frame.shape[:2]
        x, y, bw, bh = [int(v) for v in box]

        # Crop center torso patch (middle 50% width, lower 40%..90% of upper-body box)
        tx1 = max(0, x + int(bw * 0.25))
        tx2 = min(w, x + int(bw * 0.75))
        ty1 = max(0, y + int(bh * 0.42))
        ty2 = min(h, y + int(bh * 0.92))

        if tx2 - tx1 < 10 or ty2 - ty1 < 10:
            return None, None

        torso = frame[ty1:ty2, tx1:tx2]
        hsv = cv2.cvtColor(torso, cv2.COLOR_BGR2HSV)

        # Mask out extreme dark/glare pixels, keep both colored and neutral shirts
        mask = cv2.inRange(hsv, np.array([0, 15, 25]), np.array([180, 255, 245]))
        if cv2.countNonZero(mask) < 20:
            mask = None

        hist = cv2.calcHist([hsv], [0, 1], mask, [16, 12], [0, 180, 0, 256])
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)

        mean_bgr = cv2.mean(torso, mask=mask)[:3]
        if sum(mean_bgr) < 10:
            mean_bgr = cv2.mean(torso)[:3]
        rgb = (int(mean_bgr[2]), int(mean_bgr[1]), int(mean_bgr[0]))
        return hist, rgb

    def _compare_owner_hist(self, candidate_hist):
        """Return similarity score (0.0 to 1.0) against locked owner's shirt histogram."""
        if candidate_hist is None:
            return 0.5
        with self._owner_lock:
            if not self.owner_color_enabled or self.owner_hist is None:
                return 1.0
            corr = cv2.compareHist(self.owner_hist, candidate_hist, cv2.HISTCMP_CORREL)
            # Map [-1..1] correlation softly into [0..1]
            score = (corr + 1.0) * 0.5 if corr < 0 else (0.5 + 0.5 * corr)
            return float(max(0.0, min(1.0, score)))

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
        """Clear the locked shirt color so the next confirmed person becomes the owner."""
        with self._owner_lock:
            self.owner_hist = None
            self.owner_rgb = None
            self.owner_match_score = 0.0
        self._face_streak = 0
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
    # FEATURE #1: Calibrated Hand Gesture Detection (Raised Palm Beside Face)
    # =========================================================================

    def set_gesture_paused(self, paused: bool):
        """Manually set or toggle Gesture Pause state (from Web UI or gesture)."""
        self.gesture_paused = bool(paused)
        self._gesture_hold_start = None
        self._gesture_progress = 0.0
        self._last_gesture_toggle = time.time()
        state_str = "PAUSED (HOLD)" if self.gesture_paused else "RESUMED (FOLLOWING)"
        logger.info(f"✋ Follow State -> {state_str}")

    def _check_hand_gesture(self, frame):
        """
        Detect a deliberately raised open palm beside the person's head.
        To prevent false triggers on walls, doors, or hanging arms:
          1. Requires a recently detected Frontal Face (within last 0.8s) so we know
             exact head coordinates (fx, fy, fw, fh) and can calibrate to the person's
             actual face skin YCrCb color under current lighting.
          2. Searches ONLY in the raised-hand boxes strictly to the left/right of the head
             (at ear/head height, completely outside the face and above the waist).
          3. Requires open-palm geometry (convexity defects from fingers or upright palm contour)
             held steady for 1.0 second.
        """
        now = time.time()
        if not self.gesture_control_enabled or self._last_face_box is None or (now - self._last_face_time > 0.8):
            self._gesture_hand_box = None
            self._gesture_hold_start = None
            self._gesture_progress = 0.0
            return

        h, w = frame.shape[:2]
        fx, fy, fw, fh = self._last_face_box

        # Face must be large enough (within ~3m) to reliably read hand gestures
        if fw < 36 or fh < 36:
            self._gesture_hand_box = None
            self._gesture_hold_start = None
            self._gesture_progress = 0.0
            return

        # Sample the person's own face skin tone (center 50% of face) in YCrCb
        fcx1 = max(0, fx + int(fw * 0.25))
        fcx2 = min(w, fx + int(fw * 0.75))
        fcy1 = max(0, fy + int(fh * 0.25))
        fcy2 = min(h, fy + int(fh * 0.75))
        if fcx2 - fcx1 < 10 or fcy2 - fcy1 < 10:
            return

        face_ycrcb = cv2.cvtColor(frame[fcy1:fcy2, fcx1:fcx2], cv2.COLOR_BGR2YCrCb)
        y_mean, cr_mean, cb_mean = cv2.mean(face_ycrcb)[:3]

        # Build calibrated skin threshold around the person's own face chrominance
        cr_low = max(128, int(cr_mean - 16))
        cr_high = min(178, int(cr_mean + 16))
        cb_low = max(72, int(cb_mean - 16))
        cb_high = min(132, int(cb_mean + 16))
        y_low = max(35, int(y_mean - 70))

        # Define left and right raised-palm search boxes at head level (strictly outside face)
        ry1 = max(0, fy - int(fh * 0.25))
        ry2 = min(h, fy + int(fh * 1.15))

        left_rx1 = max(0, fx - int(fw * 1.45))
        left_rx2 = max(0, fx - int(fw * 0.20))

        right_rx1 = min(w, fx + fw + int(fw * 0.20))
        right_rx2 = min(w, fx + fw + int(fw * 1.45))

        found_hand = None
        for (rx1, rx2) in ((left_rx1, left_rx2), (right_rx1, right_rx2)):
            if rx2 - rx1 < 28 or ry2 - ry1 < 28:
                continue
            roi = frame[ry1:ry2, rx1:rx2]
            ycrcb = cv2.cvtColor(roi, cv2.COLOR_BGR2YCrCb)
            skin_mask = cv2.inRange(
                ycrcb,
                np.array([y_low, cr_low, cb_low], dtype=np.uint8),
                np.array([255, cr_high, cb_high], dtype=np.uint8)
            )
            skin_mask = cv2.medianBlur(skin_mask, 5)

            contours, _ = cv2.findContours(skin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            roi_area = float((rx2 - rx1) * (ry2 - ry1))

            for cnt in contours:
                area = cv2.contourArea(cnt)
                # Raised palm occupies 16% to 62% of the head-side ROI
                if roi_area * 0.16 <= area <= roi_area * 0.62:
                    hx, hy, hw, hh = cv2.boundingRect(cnt)
                    # Reject blobs touching 3 edges of ROI (which are background walls)
                    touches = (1 if hx <= 2 else 0) + (1 if hy <= 2 else 0) + \
                              (1 if hx + hw >= (rx2 - rx1 - 2) else 0) + (1 if hy + hh >= (ry2 - ry1 - 2) else 0)
                    if touches >= 3:
                        continue

                    aspect = float(hw) / float(max(1, hh))
                    hull = cv2.convexHull(cnt)
                    hull_area = max(1.0, cv2.contourArea(hull))
                    solidity = area / hull_area

                    # Open palm with spread fingers has moderate solidity (0.50..0.84) and upright shape
                    if 0.45 <= aspect <= 1.25 and 0.50 <= solidity <= 0.84:
                        found_hand = (rx1 + hx, ry1 + hy, hw, hh)
                        break
            if found_hand is not None:
                break

        if found_hand is not None:
            self._gesture_hand_box = found_hand
            if self._gesture_hold_start is None:
                self._gesture_hold_start = now
            hold_elapsed = now - self._gesture_hold_start
            self._gesture_progress = min(1.0, hold_elapsed / self.GESTURE_HOLD_SEC)

            if (hold_elapsed >= self.GESTURE_HOLD_SEC and
                    now - self._last_gesture_toggle >= self.GESTURE_COOLDOWN_SEC):
                self.gesture_paused = not self.gesture_paused
                self._last_gesture_toggle = now
                self._gesture_hold_start = None
                self._gesture_progress = 0.0
                state_txt = "✋ PAUSED (HOLDING POSITION)" if self.gesture_paused else "▶️ RESUMED FOLLOWING"
                logger.info(f"Hand Gesture Triggered -> {state_txt}")
        else:
            self._gesture_hand_box = None
            self._gesture_hold_start = None
            self._gesture_progress = 0.0

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
    # THREAD 3: On-Demand Back / Full-Body Detector (Upper-Body + Fast HOG)
    # =========================================================================

    def _background_detect_loop(self):
        """
        Runs in background ONLY when frontal face has not been seen recently (<0.4s),
        such as when the owner turns their back to walk away.
        Normalizes all bounding boxes to the exact same upper-body scale as Frontal Face
        so FollowController distance/speed PID works identically from Front or Back!
        """
        last_bg_frame_id = -1

        while self._running:
            # If Frontal Face is actively locked in Thread 2, sleep lightly and save 100% CPU
            if time.time() - self._last_face_time < 0.45 and self.is_tracking:
                time.sleep(0.12)
                continue

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

            candidates = []  # list of (tx, ty, tw, th, label) normalized to upper-body scale

            # --- Stage B: Haar Upper-Body (Head + Shoulders from BACK or FRONT) ---
            if self.upperbody_cascade is not None:
                ubodies = self.upperbody_cascade.detectMultiScale(
                    small_gray, scaleFactor=1.18, minNeighbors=4, minSize=(36, 36),
                    flags=cv2.CASCADE_SCALE_IMAGE
                )
                for (sx, sy, sw, sh) in ubodies:
                    # Normalize to torso/upper-body box matching Stage A area scale
                    ux, uy, uw, uh = sx * 2, sy * 2, sw * 2, sh * 2
                    tx = max(0, ux + int(uw * 0.12))
                    ty = max(0, uy + int(uh * 0.05))
                    tw = min(w - tx, int(uw * 0.76))
                    th = min(h - ty, int(uh * 1.15))
                    candidates.append((tx, ty, tw, th, "BACK/UPPER-BODY"))

            # --- Stage C: Fast HOG Full-Body Detector (240x180 for <45ms latency on RPi 4) ---
            if not candidates and self.hog is not None:
                hog_w, hog_h = 240, 180
                hog_gray = cv2.resize(small_gray, (hog_w, hog_h), interpolation=cv2.INTER_AREA)
                rects, weights = self.hog.detectMultiScale(
                    hog_gray, winStride=(8, 8), padding=(8, 8), scale=1.12
                )
                scale_x = float(w) / float(hog_w)
                scale_y = float(h) / float(hog_h)

                for i, (rx, ry, rw, rh) in enumerate(rects):
                    if weights[i] >= 0.45:
                        fx = int(rx * scale_x)
                        fy = int(ry * scale_y)
                        fw = int(rw * scale_x)
                        fh = int(rh * scale_y)
                        # Crop full-body HOG box to the upper-body/torso core so area_ratio
                        # matches TARGET_AREA_RATIO (0.16) and TOO_CLOSE_STOP_RATIO (0.28)!
                        tx = max(0, fx + int(fw * 0.22))
                        ty = max(0, fy + int(fh * 0.12))
                        tw = min(w - tx, int(fw * 0.56))
                        th = min(h - ty, int(fh * 0.48))
                        candidates.append((tx, ty, tw, th, "FULL-BODY (BACK/SIDE)"))

            # Score candidates with Owner Shirt-Color Lock
            best_cand = None
            best_score = -1.0

            with self._owner_lock:
                has_owner = self.owner_color_enabled and (self.owner_hist is not None)

            for (tx, ty, tw, th, label) in candidates:
                area_ratio = float(tw * th) / float(w * h)
                # Ignore impossibly tiny or huge false-positive boxes
                if area_ratio < 0.02 or area_ratio > 0.45:
                    continue

                hist, rgb = self._extract_torso_hist(frame, (tx, ty, tw, th))
                color_match = self._compare_owner_hist(hist)

                # For non-face background detectors (Upper-Body/HOG), require decent shirt match
                # if an owner is already locked (prevents locking onto chairs/walls)
                if has_owner and color_match < self.OWNER_MIN_MATCH:
                    continue

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
                with self._bg_lock:
                    self._bg_candidate = best_cand

            time.sleep(0.12)

    # =========================================================================
    # THREAD 2: Fast Frontal Face + 30 FPS Tracker + HUD Overlay
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

    def _detect_frontal_face_torso(self, frame):
        """
        Fast synchronous half-resolution (320x240) Haar Frontal Face detector (~6ms).
        Expands detected face into an upper-body torso box and scores with Owner Shirt Lock.
        """
        h, w = frame.shape[:2]
        small_gray = cv2.resize(frame, (w // 2, h // 2), interpolation=cv2.INTER_LINEAR)
        small_gray = cv2.cvtColor(small_gray, cv2.COLOR_BGR2GRAY)

        faces = self.face_cascade.detectMultiScale(
            small_gray,
            scaleFactor=1.2,
            minNeighbors=4,
            minSize=(18, 18),
            flags=cv2.CASCADE_SCALE_IMAGE
        )

        if len(faces) == 0:
            return None

        with self._owner_lock:
            has_owner = self.owner_color_enabled and (self.owner_hist is not None)

        best_cand = None
        best_score = -1.0
        num_faces = len(faces)

        for (sx, sy, sw, sh) in faces:
            fx, fy, fw, fh = sx * 2, sy * 2, sw * 2, sh * 2
            pad_x = int(fw * 0.25)
            pad_top = int(fh * 0.10)
            pad_bot = int(fh * 1.40)

            tx = max(0, fx - pad_x)
            ty = max(0, fy - pad_top)
            tw = min(w - tx, fw + 2 * pad_x)
            th = min(h - ty, fh + pad_top + pad_bot)

            hist, rgb = self._extract_torso_hist(frame, (tx, ty, tw, th))
            color_match = self._compare_owner_hist(hist)

            # If multiple people are facing the camera and owner is locked, reject non-matching shirts
            if has_owner and num_faces > 1 and color_match < self.OWNER_MIN_MATCH:
                continue

            area_ratio = float(tw * th) / float(w * h)
            score = (color_match * 2.5) + area_ratio if has_owner else area_ratio

            if score > best_score:
                best_score = score
                best_cand = {
                    'face_box': (fx, fy, fw, fh),
                    'box': (tx, ty, tw, th),
                    'label': "FACE+TORSO",
                    'hist': hist,
                    'rgb': rgb,
                    'color_match': color_match
                }

        return best_cand

    def _run_detection(self, frame):
        h, w = frame.shape[:2]
        annotated = frame.copy()
        best_detection = None

        # =====================================================================
        # STEP 1: Fast Tracking (runs at 30 FPS between periodic re-anchors)
        # =====================================================================
        if self.is_tracking and self.tracker is not None:
            success, box = self.tracker.update(frame)
            if success:
                x, y, box_w, box_h = [int(v) for v in box]
                x = max(0, min(w - 10, x))
                y = max(0, min(h - 10, y))
                box_w = max(10, min(w - x, box_w))
                box_h = max(10, min(h - y, box_h))

                # Update Owner Shirt-Color match score every 6 frames for HUD
                if self.frames_since_detect % 6 == 0:
                    hist, _ = self._extract_torso_hist(frame, (x, y, box_w, box_h))
                    match = self._compare_owner_hist(hist)
                    with self._owner_lock:
                        self.owner_match_score = match

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
                    'confidence': max(0.6, self.owner_match_score),
                    'gesture_paused': self.gesture_paused
                }

                box_color = (0, 180, 255) if self.gesture_paused else (0, 255, 255)
                cv2.rectangle(annotated, (x, y), (x + box_w, y + box_h), box_color, 2)
                cv2.putText(
                    annotated,
                    f"{self._last_detect_label} ({int(self.owner_match_score * 100)}%)",
                    (x, max(18, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2
                )

                self.frames_since_detect += 1
                if self.frames_since_detect >= self.MAX_TRACK_FRAMES:
                    self.is_tracking = False
            else:
                self.is_tracking = False

        # =====================================================================
        # STEP 2: Detect / Re-Anchor Target (when not tracking or every 30 frames)
        #   2A. Try fast Frontal Face first (~6ms in Thread 2)
        #   2B. If no frontal face, check Thread 3 Back/Upper-Body/Full-Body candidate
        # =====================================================================
        if not self.is_tracking:
            cand = self._detect_frontal_face_torso(frame)

            if cand is not None:
                self._last_face_box = cand['face_box']
                self._last_face_time = time.time()
                self._face_streak += 1

                # Auto-lock Owner Shirt Color after 4 confirmed frontal-face detections
                with self._owner_lock:
                    if self.owner_color_enabled and self.owner_hist is None and self._face_streak >= 4 and cand['hist'] is not None:
                        self.owner_hist = cand['hist']
                        self.owner_rgb = cand['rgb']
                        self.owner_match_score = 1.0
                        cand['color_match'] = 1.0
                        logger.info(f"👕 Auto-locked Owner Shirt Color from confirmed face: RGB={self.owner_rgb}")
                    elif self.owner_hist is not None and cand['hist'] is not None and cand['color_match'] > 0.65:
                        # Gently adapt owner histogram (5%) to smooth indoor/outdoor lighting shifts
                        cv2.addWeighted(self.owner_hist, 0.95, cand['hist'], 0.05, 0, self.owner_hist)
            else:
                # No frontal face — check if Thread 3 found the person from BACK / SIDE
                with self._bg_lock:
                    if self._bg_candidate is not None and (time.time() - self._bg_candidate['time'] < 0.65):
                        cand = self._bg_candidate
                        self._bg_candidate = None

            if cand is not None:
                tx, ty, tw, th = cand['box']
                self._last_detect_label = cand['label']
                with self._owner_lock:
                    self.owner_match_score = cand['color_match']

                area = tw * th
                best_detection = {
                    'cx': tx + (tw // 2),
                    'cy': ty + (th // 2),
                    'w': tw, 'h': th,
                    'x1': tx, 'y1': ty,
                    'x2': tx + tw, 'y2': ty + th,
                    'area': area,
                    'area_ratio': float(area) / float(w * h),
                    'confidence': max(0.6, cand['color_match']),
                    'gesture_paused': self.gesture_paused
                }

                cv2.rectangle(annotated, (tx, ty), (tx + tw, ty + th), (0, 0, 255), 2)
                cv2.putText(
                    annotated,
                    f"DETECTED: {cand['label']}",
                    (tx, max(18, ty - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2
                )

                self.tracker = self._create_tracker()
                self.tracker.init(frame, (tx, ty, tw, th))
                self.is_tracking = True
                self.frames_since_detect = 0

        # =====================================================================
        # STEP 3: Check Calibrated Hand Gesture (when face was recently seen)
        # =====================================================================
        if best_detection is not None:
            self._check_hand_gesture(frame)
            best_detection['gesture_paused'] = self.gesture_paused
        else:
            self._gesture_hand_box = None
            self._gesture_hold_start = None
            self._gesture_progress = 0.0

        # Draw Raised Hand Gesture Box + Hold Progress Bar if detected
        if self._gesture_hand_box is not None:
            hx, hy, hw, hh = self._gesture_hand_box
            cv2.rectangle(annotated, (hx, hy), (hx + hw, hy + hh), (255, 0, 255), 2)
            pct = int(self._gesture_progress * 100)
            cv2.putText(annotated, f"HAND HOLD {pct}%", (hx, max(15, hy - 6)),
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
