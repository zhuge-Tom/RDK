#!/usr/bin/env python3
"""Calibrate intrinsics from raw chessboard PNGs; leave the result unverified."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, default=Path("calibration/accepted"),
                        help="reviewed image directory (default: %(default)s)")
    parser.add_argument("--columns", type=int, required=True, help="inner corners horizontally")
    parser.add_argument("--rows", type=int, required=True, help="inner corners vertically")
    parser.add_argument("--square-mm", type=float, default=19.0,
                        help="measured square edge in mm (current printed board; default: %(default)s)")
    parser.add_argument("--min-views", type=int, default=15)
    parser.add_argument("--output", type=Path, default=Path("config/pose_camera.measured.yml"))
    args = parser.parse_args()
    if args.columns < 3 or args.rows < 3 or not np.isfinite(args.square_mm) or args.square_mm <= 0:
        parser.error("use at least 3 x 3 inner corners and a positive measured square size")
    if args.min_views < 10:
        parser.error("at least 10 usable views are required; 20-30 varied views are recommended")
    if args.output.exists() or args.output.with_suffix(".report.json").exists():
        parser.error("output already exists; choose a new output filename to preserve the previous calibration")

    pattern = (args.columns, args.rows)
    grid = np.zeros((args.columns * args.rows, 3), np.float32)
    grid[:, :2] = np.mgrid[0:args.columns, 0:args.rows].T.reshape(-1, 2) * (args.square_mm / 1000)
    points, paths, rejected = [], [], []
    size = None
    for path in sorted(args.images.glob("*.png")):
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            rejected.append({"file": path.name, "reason": "unreadable"})
            continue
        current_size = (gray.shape[1], gray.shape[0])
        if size is not None and size != current_size:
            raise SystemExit("Mixed image resolutions; use images from one camera configuration only")
        size = current_size
        if hasattr(cv2, "findChessboardCornersSB"):
            found, corners = cv2.findChessboardCornersSB(gray, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE)
        else:
            found, corners = cv2.findChessboardCorners(gray, pattern)
            if found:
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                          (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, .001))
        if found:
            # OpenCV versions return either (N, 2) or (N, 1, 2).
            points.append(np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2))
            paths.append(path.name)
        else:
            rejected.append({"file": path.name, "reason": "chessboard_not_found"})
        print(f"{'OK' if found else 'SKIP'} {path.name}", flush=True)
    if len(points) < args.min_views:
        raise SystemExit(f"Only {len(points)} usable views; need at least {args.min_views}. "
                         "Check inner corner counts, focus, exposure and full board visibility.")

    rms, matrix, distortion, rotations, translations = cv2.calibrateCamera(
        [grid] * len(points), points, size, None, None)
    if not np.isfinite(rms) or not np.isfinite(matrix).all() or not np.isfinite(distortion).all():
        raise SystemExit("Calibration produced invalid values; recapture varied board poses")
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise SystemExit("Invalid focal length; check board geometry and capture diversity")
    errors = []
    for name, corners, rotation, translation in zip(paths, points, rotations, translations):
        projected, _ = cv2.projectPoints(grid, rotation, translation, matrix, distortion)
        residual = projected.reshape(-1, 2) - corners.reshape(-1, 2)
        error = float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))
        errors.append({"file": name, "rms_px": error})
    checked_rms = float(np.sqrt(np.mean([entry["rms_px"] ** 2 for entry in errors])))
    if not np.isfinite(checked_rms) or not np.isclose(checked_rms, rms, rtol=.001, atol=.001):
        raise SystemExit("Per-view reprojection errors disagree with calibration RMS; result was not saved")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    storage = cv2.FileStorage(str(args.output), cv2.FILE_STORAGE_WRITE)
    if not storage.isOpened():
        raise SystemExit(f"Cannot write {args.output}")
    try:
        storage.write("calibration_verified", 0)
        storage.write("image_width", size[0])
        storage.write("image_height", size[1])
        storage.write("camera_matrix", matrix)
        storage.write("distortion_coefficients", distortion.reshape(1, -1))
        storage.write("armor_width_m", 0.)
        storage.write("armor_height_m", 0.)
        storage.write("max_reprojection_error_px", 4.)
        storage.write("calibration_rms_px", rms)
        storage.write("calibration_views", len(points))
    finally:
        storage.release()
    report = dict(image_width=size[0], image_height=size[1],
                  board_inner_corners=list(pattern), square_mm=args.square_mm,
                  rms_px=rms, views=errors, rejected=rejected,
                  calibration_verified=False,
                  next_step="Review pose coverage and reprojection errors, measure actual lightbar "
                            "dimensions, then verify distances with independent measurements.")
    report_path = args.output.with_suffix(".report.json")
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(f"Saved {args.output} and {report_path}; {len(points)} views, RMS {rms:.3f} px")
    print("Result remains unverified; real lightbar dimensions and distance checks are still required.")


if __name__ == "__main__":
    main()
