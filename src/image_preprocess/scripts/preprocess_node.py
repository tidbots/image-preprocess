#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from dataclasses import dataclass

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Import custom message (generated at build time)
try:
    from image_preprocess.msg import PreprocessStats
    HAS_STATS_MSG = True
except ImportError:
    HAS_STATS_MSG = False
    rospy.logwarn("PreprocessStats message not found. Stats publishing disabled.")


# Preset definitions
PRESETS = {
    "dark_venue": {
        "gamma": 1.4,
        "clahe_clip": 3.0,
        "auto_tune_enable": False,
    },
    "bright_venue": {
        "gamma": 0.85,
        "clahe_clip": 1.5,
        "auto_tune_enable": False,
    },
    "uniform": {
        "gamma": 1.0,
        "clahe_clip": 2.0,
        "auto_tune_enable": False,
    },
    "auto": {
        "gamma": 1.1,
        "clahe_clip": 2.5,
        "auto_tune_enable": True,
    },
}


@dataclass
class BrightnessStats:
    mean: float = 0.0
    std: float = 0.0
    sat_ratio: float = 0.0  # ratio of pixels > sat_thr
    dark_ratio: float = 0.0  # ratio of pixels < dark_thr


class PreprocessNode:
    """
    Image preprocess for illumination robustness:
      - Gamma correction (optional)
      - CLAHE on L channel (optional)

    Features:
      - Preset configurations for common lighting scenarios
      - Auto re-tuning with EMA smoothing
      - Stats publishing for monitoring
      - Image quality assessment (blur, exposure)
      - Histogram overlay for debugging
    """

    def __init__(self):
        rospy.init_node("image_preprocess_node")

        # Load preset first (if specified)
        self._load_preset()

        # Topics
        self.input_topic = rospy.get_param("~input_topic", "/camera/image_raw")
        self.output_topic = rospy.get_param("~output_topic", "/camera/image_preprocessed")
        self.debug_topic = rospy.get_param("~debug_topic", "/camera/image_preprocess_debug")
        self.stats_topic = rospy.get_param("~stats_topic", "/camera/image_preprocess_stats")

        # Enable flags
        self.debug_enable = bool(rospy.get_param("~debug_enable", False))
        self.debug_histogram = bool(rospy.get_param("~debug_histogram", False))
        self.stats_enable = bool(rospy.get_param("~stats_enable", True))
        self.gamma_enable = bool(rospy.get_param("~gamma_enable", True))
        self.clahe_enable = bool(rospy.get_param("~clahe_enable", True))

        # Preprocess base params (may be overridden by preset)
        self.gamma = float(rospy.get_param("~gamma", self._preset_defaults.get("gamma", 1.10)))
        self.clahe_clip = float(rospy.get_param("~clahe_clip", self._preset_defaults.get("clahe_clip", 2.5)))
        self.clahe_grid = int(rospy.get_param("~clahe_grid", 8))

        # Auto tuning enable (may be overridden by preset)
        self.auto_tune_enable = bool(rospy.get_param(
            "~auto_tune_enable",
            self._preset_defaults.get("auto_tune_enable", False)
        ))

        # Auto tuning targets & thresholds
        self.target_mean = float(rospy.get_param("~target_mean", 125.0))
        self.dark_mean_thr = float(rospy.get_param("~dark_mean_thr", 90.0))
        self.bright_mean_thr = float(rospy.get_param("~bright_mean_thr", 170.0))
        self.low_contrast_std_thr = float(rospy.get_param("~low_contrast_std_thr", 35.0))

        self.sat_thr = int(rospy.get_param("~sat_thr", 245))
        self.dark_thr = int(rospy.get_param("~dark_thr", 10))
        self.sat_ratio_thr = float(rospy.get_param("~sat_ratio_thr", 0.12))
        self.dark_ratio_thr = float(rospy.get_param("~dark_ratio_thr", 0.12))

        # Auto tuning step sizes
        self.gamma_step = float(rospy.get_param("~gamma_step", 0.05))
        self.gamma_step_saturated = float(rospy.get_param("~gamma_step_saturated", 0.08))
        self.clahe_step = float(rospy.get_param("~clahe_step", 0.2))

        # Bounds
        self.gamma_min = float(rospy.get_param("~gamma_min", 0.70))
        self.gamma_max = float(rospy.get_param("~gamma_max", 1.60))
        self.clahe_min = float(rospy.get_param("~clahe_min", 1.2))
        self.clahe_max = float(rospy.get_param("~clahe_max", 3.8))

        # Rate limiting
        self.update_every_n = int(rospy.get_param("~auto_tune_update_every_n", 8))
        self.min_update_interval = float(rospy.get_param("~auto_tune_min_update_interval", 0.25))

        # Smoothing (EMA)
        self.ema_alpha = float(rospy.get_param("~ema_alpha", 0.15))

        # Performance monitoring
        self.warn_frame_time_ms = float(rospy.get_param("~warn_frame_time_ms", 30.0))

        # Quality thresholds
        self.blur_threshold = float(rospy.get_param("~blur_threshold", 100.0))
        self.overexpose_ratio = float(rospy.get_param("~overexpose_ratio", 0.3))
        self.underexpose_ratio = float(rospy.get_param("~underexpose_ratio", 0.3))

        # Validate parameters
        self._validate_params()

        # Publishers and subscribers
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.pub_dbg = rospy.Publisher(self.debug_topic, Image, queue_size=1) if self.debug_enable else None

        if self.stats_enable and HAS_STATS_MSG:
            self.pub_stats = rospy.Publisher(self.stats_topic, PreprocessStats, queue_size=1)
        else:
            self.pub_stats = None

        self.sub = rospy.Subscriber(self.input_topic, Image, self.cb, queue_size=1)

        # State
        self._frame = 0
        self._last_update_t = 0.0
        self._ema = BrightnessStats()
        self._last_blur_score = 0.0
        self._last_frame_time_ms = 0.0

        # Log startup info
        rospy.loginfo("image_preprocess ready:")
        rospy.loginfo("  preset=%s", self._preset_name)
        rospy.loginfo("  in=%s out=%s", self.input_topic, self.output_topic)
        rospy.loginfo("  gamma=%.2f (enable=%s), clahe=%.2f (enable=%s)",
                      self.gamma, self.gamma_enable, self.clahe_clip, self.clahe_enable)
        rospy.loginfo("  auto_tune=%s, stats=%s, debug=%s",
                      self.auto_tune_enable, self.stats_enable, self.debug_enable)

    def _load_preset(self):
        """Load preset configuration if specified."""
        self._preset_name = rospy.get_param("~preset", "none")
        self._preset_defaults = {}

        if self._preset_name in PRESETS:
            self._preset_defaults = PRESETS[self._preset_name].copy()
            rospy.loginfo("Loading preset: %s", self._preset_name)
        elif self._preset_name != "none":
            rospy.logwarn("Unknown preset '%s'. Available: %s",
                          self._preset_name, list(PRESETS.keys()))

    def _validate_params(self):
        """Validate parameters and log warnings for out-of-range values."""
        # Gamma validation
        if self.gamma < 0.5 or self.gamma > 2.0:
            rospy.logwarn("gamma=%.2f is outside recommended range [0.5, 2.0]", self.gamma)
        if self.gamma <= 0:
            rospy.logerr("gamma must be > 0. Setting to 1.0")
            self.gamma = 1.0

        # CLAHE validation
        if self.clahe_clip < 1.0:
            rospy.logwarn("clahe_clip=%.2f is < 1.0, may cause issues. Setting to 1.0", self.clahe_clip)
            self.clahe_clip = 1.0
        if self.clahe_clip > 10.0:
            rospy.logwarn("clahe_clip=%.2f is very high, may cause noise", self.clahe_clip)

        if self.clahe_grid < 2:
            rospy.logwarn("clahe_grid=%d is < 2, setting to 2", self.clahe_grid)
            self.clahe_grid = 2

        # Bounds validation
        if self.gamma_min >= self.gamma_max:
            rospy.logerr("gamma_min >= gamma_max. Using defaults.")
            self.gamma_min = 0.70
            self.gamma_max = 1.60

        if self.clahe_min >= self.clahe_max:
            rospy.logerr("clahe_min >= clahe_max. Using defaults.")
            self.clahe_min = 1.2
            self.clahe_max = 3.8

        # EMA validation
        if self.ema_alpha <= 0 or self.ema_alpha > 1:
            rospy.logwarn("ema_alpha=%.2f is invalid. Setting to 0.15", self.ema_alpha)
            self.ema_alpha = 0.15

    def _compute_stats(self, bgr: np.ndarray) -> BrightnessStats:
        """Compute brightness statistics from image."""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        mean = float(gray.mean())
        std = float(gray.std())
        sat_ratio = float(np.mean(gray > self.sat_thr))
        dark_ratio = float(np.mean(gray < self.dark_thr))
        return BrightnessStats(mean=mean, std=std, sat_ratio=sat_ratio, dark_ratio=dark_ratio)

    def _compute_blur_score(self, bgr: np.ndarray) -> float:
        """Compute blur score using Laplacian variance. Higher = sharper."""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        return float(cv2.Laplacian(gray, cv2.CV_64F).var())

    def _ema_update(self, s: BrightnessStats):
        """Update EMA statistics."""
        a = self.ema_alpha
        self._ema.mean = (1 - a) * self._ema.mean + a * s.mean
        self._ema.std = (1 - a) * self._ema.std + a * s.std
        self._ema.sat_ratio = (1 - a) * self._ema.sat_ratio + a * s.sat_ratio
        self._ema.dark_ratio = (1 - a) * self._ema.dark_ratio + a * s.dark_ratio

    def _auto_tune(self):
        """
        Adjust gamma & CLAHE based on EMA stats.
        Safe, small steps, bounded, rate-limited.
        """
        now = time.time()
        if (self._frame % self.update_every_n) != 0:
            return
        if (now - self._last_update_t) < self.min_update_interval:
            return

        mean = self._ema.mean
        std = self._ema.std
        sat = self._ema.sat_ratio
        dark = self._ema.dark_ratio

        gamma = self.gamma
        clahe = self.clahe_clip

        # Priority: saturation (white-out) first
        if sat > self.sat_ratio_thr:
            gamma = max(self.gamma_min, gamma - self.gamma_step_saturated)
            clahe = max(self.clahe_min, clahe - self.clahe_step)
        else:
            # Dark / bright control
            if mean < self.dark_mean_thr or dark > self.dark_ratio_thr:
                gamma = min(self.gamma_max, gamma + self.gamma_step)
            elif mean > self.bright_mean_thr:
                gamma = max(self.gamma_min, gamma - self.gamma_step)

            # Contrast control (gentle)
            if std < self.low_contrast_std_thr:
                clahe = min(self.clahe_max, clahe + self.clahe_step)
            else:
                target = 2.3
                if clahe > target + 0.2:
                    clahe = max(self.clahe_min, clahe - self.clahe_step * 0.5)
                elif clahe < target - 0.2:
                    clahe = min(self.clahe_max, clahe + self.clahe_step * 0.3)

        # Apply
        if (abs(gamma - self.gamma) > 1e-6) or (abs(clahe - self.clahe_clip) > 1e-6):
            self.gamma = float(np.clip(gamma, self.gamma_min, self.gamma_max))
            self.clahe_clip = float(np.clip(clahe, self.clahe_min, self.clahe_max))
            self._last_update_t = now

            rospy.loginfo_throttle(
                1.0,
                "auto_tune: gamma=%.2f clahe=%.2f (mean=%.1f std=%.1f sat=%.2f dark=%.2f)",
                self.gamma, self.clahe_clip, mean, std, sat, dark
            )

    @staticmethod
    def _apply_gamma(bgr: np.ndarray, gamma: float) -> np.ndarray:
        """Apply gamma correction using LUT."""
        if gamma <= 0:
            return bgr
        inv = 1.0 / gamma
        table = (np.linspace(0, 1, 256) ** inv * 255.0).astype(np.uint8)
        return cv2.LUT(bgr, table)

    @staticmethod
    def _apply_clahe(bgr: np.ndarray, clip: float, grid: int) -> np.ndarray:
        """Apply CLAHE on L channel in LAB color space."""
        grid = max(2, int(grid))
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        l_ch, a_ch, b_ch = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=float(clip), tileGridSize=(grid, grid))
        l_ch = clahe.apply(l_ch)
        lab = cv2.merge([l_ch, a_ch, b_ch])
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    def _draw_histogram(self, img: np.ndarray, x: int, y: int, w: int, h: int) -> np.ndarray:
        """Draw histogram overlay on image."""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
        hist = hist.flatten()
        hist_max = hist.max() if hist.max() > 0 else 1

        # Draw background
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 0), -1)
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 255), 1)

        # Draw histogram bars
        for i in range(256):
            bar_h = int(hist[i] / hist_max * (h - 4))
            if bar_h > 0:
                bx = x + 2 + i * (w - 4) // 256
                cv2.line(img, (bx, y + h - 2), (bx, y + h - 2 - bar_h), (0, 255, 0), 1)

        return img

    def _publish_stats(self, header, frame_time_ms: float):
        """Publish statistics message."""
        if self.pub_stats is None:
            return

        msg = PreprocessStats()
        msg.header = header
        msg.mean_luma = self._ema.mean
        msg.std_luma = self._ema.std
        msg.sat_ratio = self._ema.sat_ratio
        msg.dark_ratio = self._ema.dark_ratio
        msg.current_gamma = self.gamma
        msg.current_clahe_clip = self.clahe_clip
        msg.frame_time_ms = frame_time_ms
        msg.blur_score = self._last_blur_score
        msg.is_overexposed = self._ema.sat_ratio > self.overexpose_ratio
        msg.is_underexposed = self._ema.dark_ratio > self.underexpose_ratio

        self.pub_stats.publish(msg)

    def cb(self, msg: Image):
        """Image callback - main processing pipeline."""
        self._frame += 1
        t_start = time.time()

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge error: %s", e)
            return

        # Compute statistics on raw image
        s = self._compute_stats(bgr)
        self._ema_update(s)

        # Compute blur score (periodically to save CPU)
        if self._frame % 5 == 0:
            self._last_blur_score = self._compute_blur_score(bgr)

        # Auto tune if enabled
        if self.auto_tune_enable:
            self._auto_tune()

        # Apply preprocessing
        out = bgr
        if self.gamma_enable:
            out = self._apply_gamma(out, self.gamma)
        if self.clahe_enable:
            out = self._apply_clahe(out, self.clahe_clip, self.clahe_grid)

        # Publish output
        out_msg = self.bridge.cv2_to_imgmsg(out, "bgr8")
        out_msg.header = msg.header
        self.pub.publish(out_msg)

        # Compute frame time
        t_end = time.time()
        frame_time_ms = (t_end - t_start) * 1000.0
        self._last_frame_time_ms = frame_time_ms

        # Warn if processing is slow
        if frame_time_ms > self.warn_frame_time_ms:
            rospy.logwarn_throttle(5.0, "Processing slow: %.1f ms (threshold: %.1f ms)",
                                   frame_time_ms, self.warn_frame_time_ms)

        # Publish stats
        if self.stats_enable:
            self._publish_stats(msg.header, frame_time_ms)

        # Debug overlay
        if self.debug_enable and self.pub_dbg is not None:
            dbg = out.copy()

            # Text overlay
            y_pos = 25
            cv2.putText(dbg, f"gamma={self.gamma:.2f} clahe={self.clahe_clip:.2f} grid={self.clahe_grid}",
                        (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
            y_pos += 25
            cv2.putText(dbg, f"mean={self._ema.mean:.1f} std={self._ema.std:.1f} sat={self._ema.sat_ratio:.2f} dark={self._ema.dark_ratio:.2f}",
                        (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
            y_pos += 20
            cv2.putText(dbg, f"blur={self._last_blur_score:.1f} time={frame_time_ms:.1f}ms",
                        (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

            # Quality warnings
            y_pos += 20
            warnings = []
            if self._ema.sat_ratio > self.overexpose_ratio:
                warnings.append("OVEREXPOSED")
            if self._ema.dark_ratio > self.underexpose_ratio:
                warnings.append("UNDEREXPOSED")
            if self._last_blur_score < self.blur_threshold:
                warnings.append("BLURRY")
            if warnings:
                cv2.putText(dbg, " ".join(warnings),
                            (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

            # Histogram overlay
            if self.debug_histogram:
                h, w = dbg.shape[:2]
                hist_w, hist_h = 200, 80
                self._draw_histogram(dbg, w - hist_w - 10, 10, hist_w, hist_h)

            dbg_msg = self.bridge.cv2_to_imgmsg(dbg, "bgr8")
            dbg_msg.header = msg.header
            self.pub_dbg.publish(dbg_msg)


if __name__ == "__main__":
    PreprocessNode()
    rospy.spin()
