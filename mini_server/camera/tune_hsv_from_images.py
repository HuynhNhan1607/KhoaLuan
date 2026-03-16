#!/usr/bin/env python3
"""
Tune yellow-box HSV thresholds from images in ./image.

Usage:
  python3 tune_hsv_from_images.py
  python3 tune_hsv_from_images.py --dir image
"""

import argparse
import glob
import os
from typing import List, Tuple

import cv2
import numpy as np


def contour_score(cnt: np.ndarray, w: int, h: int) -> float:
    """Score contour by area and proximity to frame center."""
    area = cv2.contourArea(cnt)
    if area <= 0:
        return -1.0
    m = cv2.moments(cnt)
    if m["m00"] <= 1e-6:
        return -1.0
    cx = m["m10"] / m["m00"]
    cy = m["m01"] / m["m00"]
    dx = (cx - (w * 0.5)) / max(w * 0.5, 1.0)
    dy = (cy - (h * 0.55)) / max(h * 0.55, 1.0)
    center_penalty = np.sqrt(dx * dx + dy * dy)
    return float(area) * (1.0 - min(center_penalty, 1.0) * 0.6)


def collect_hsv_samples(img: np.ndarray) -> np.ndarray:
    """Collect HSV samples from the dominant yellow blob in one image."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Wide pre-mask for yellow-ish candidates.
    mask = cv2.inRange(hsv, (10, 10, 20), (60, 255, 255))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return np.empty((0, 3), dtype=np.uint8)

    h, w = img.shape[:2]
    frame_area = float(h * w)
    filtered = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < frame_area * 0.003:
            continue
        # Reject contours too large (usually floor/background washout).
        if area > frame_area * 0.65:
            continue
        filtered.append(c)

    if not filtered:
        return np.empty((0, 3), dtype=np.uint8)

    best = max(filtered, key=lambda c: contour_score(c, w, h))

    blob_mask = np.zeros(mask.shape, dtype=np.uint8)
    cv2.drawContours(blob_mask, [best], -1, 255, thickness=-1)

    # Trim blob border where highlights/shadows are strongest.
    erode_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    blob_mask = cv2.erode(blob_mask, erode_k, iterations=1)

    pixels = hsv[blob_mask > 0]
    if pixels.size == 0:
        return pixels

    # Remove gray/dark outliers inside contour.
    sv_ok = (pixels[:, 1] >= 20) & (pixels[:, 2] >= 30)
    pixels = pixels[sv_ok]
    return pixels


def robust_limits(values: np.ndarray, lo_pct: float, hi_pct: float) -> Tuple[int, int]:
    lo = int(np.percentile(values, lo_pct))
    hi = int(np.percentile(values, hi_pct))
    return lo, hi


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", default="image", help="Image directory (default: image)")
    args = parser.parse_args()

    patterns = ["*.jpg", "*.jpeg", "*.png", "*.bmp", "*.webp"]
    files: List[str] = []
    for p in patterns:
        files.extend(glob.glob(os.path.join(args.dir, p)))

    files = sorted(set(files))
    if not files:
        raise SystemExit(f"No images found in: {args.dir}")

    all_pixels = []
    used_files = 0
    for f in files:
        img = cv2.imread(f)
        if img is None:
            continue
        px = collect_hsv_samples(img)
        if px.size == 0:
            continue
        all_pixels.append(px)
        used_files += 1

    if not all_pixels:
        raise SystemExit("Could not extract yellow blob samples from images.")

    pixels = np.vstack(all_pixels)
    h = pixels[:, 0]
    s = pixels[:, 1]
    v = pixels[:, 2]

    # Robust percentiles to reduce outlier impact.
    h_lo, h_hi = robust_limits(h, 3, 97)
    s_lo, _ = robust_limits(s, 8, 99)
    v_lo, _ = robust_limits(v, 8, 99)

    # Safety margins + hard floors to avoid unusable thresholds.
    h_lo = max(10, h_lo - 2)
    h_hi = min(60, h_hi + 2)
    s_lo = max(20, s_lo - 10)
    v_lo = max(30, v_lo - 10)

    print("[HSV TUNE] Images scanned:", len(files))
    print("[HSV TUNE] Images used   :", used_files)
    print("[HSV TUNE] Samples       :", len(pixels))
    print()
    print("Suggested settings:")
    print(f"  YELLOW_H_LO={h_lo}")
    print(f"  YELLOW_H_HI={h_hi}")
    print(f"  YELLOW_S_LO={s_lo}")
    print(f"  YELLOW_V_LO={v_lo}")
    print()
    print("Run example:")
    print(
        f"  YELLOW_H_LO={h_lo} YELLOW_H_HI={h_hi} "
        f"YELLOW_S_LO={s_lo} YELLOW_V_LO={v_lo} ./pose_estimate_service 0"
    )


if __name__ == "__main__":
    main()
