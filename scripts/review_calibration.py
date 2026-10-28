#!/usr/bin/env python3
"""Review original chessboard PNGs; optionally copy accepted files and quarantine rejects."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, default=Path("calibration/captures"))
    parser.add_argument("--accepted", type=Path, default=Path("calibration/accepted"))
    parser.add_argument("--quarantine", type=Path, default=Path("calibration/quarantine"))
    parser.add_argument("--report", type=Path, default=Path("calibration/review.json"))
    parser.add_argument("--columns", type=int, default=9, help="horizontal inner corners")
    parser.add_argument("--rows", type=int, default=6, help="vertical inner corners")
    parser.add_argument("--max-edge-px", type=float, default=6., help="relaxed initial-prototype cutoff; prefer about 3 px")
    parser.add_argument("--min-contrast", type=float, default=60.)
    parser.add_argument("--min-coverage-percent", type=float, default=3.)
    parser.add_argument("--apply", action="store_true", help="copy selected images and move rejected originals")
    args = parser.parse_args()
    if args.columns < 3 or args.rows < 3:
        parser.error("at least 3 x 3 inner corners are required")
    if (not np.isfinite([args.max_edge_px, args.min_contrast, args.min_coverage_percent]).all()
            or args.max_edge_px <= 0 or not 0 <= args.min_contrast <= 255
            or not 0 <= args.min_coverage_percent <= 100):
        parser.error("invalid quality thresholds")
    if not hasattr(cv2, "estimateChessboardSharpness"):
        parser.error("this review tool requires OpenCV estimateChessboardSharpness")
    paths = sorted(args.images.glob("*.png"), key=lambda path: (path.stat().st_mtime_ns, path.name))
    if not paths:
        parser.error("no PNGs found in the capture directory")
    source_root, accepted_root, quarantine_root = (p.resolve() for p in (args.images, args.accepted, args.quarantine))
    roots = (source_root, accepted_root, quarantine_root)
    if any(a == b or a in b.parents or b in a.parents for i, a in enumerate(roots) for b in roots[i+1:]):
        parser.error("capture, accepted and quarantine directories must be separate, non-nested paths")
    if args.report.exists():
        parser.error("report already exists; choose a new --report filename")
    if args.apply and (list(args.accepted.glob("*.png")) or (args.accepted / "selection.json").exists()):
        parser.error("accepted set already exists; use a fresh directory to preserve the previous review")
    if args.apply and any((args.quarantine / path.name).exists() for path in paths):
        parser.error("a quarantine filename already exists; use a fresh quarantine directory")

    cv2.setNumThreads(1)
    pattern = (args.columns, args.rows)
    grid = np.mgrid[0:args.columns, 0:args.rows].T.reshape(-1, 2).astype(np.float32)
    reference_size = None
    records = []
    for index, path in enumerate(paths, 1):
        row = dict(index=index, filename=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(), reasons=[])
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            row["reasons"].append("unreadable_image")
        else:
            height, width = gray.shape
            size = (width, height)
            row.update(width=width, height=height)
            found, corners = cv2.findChessboardCornersSB(gray, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE)
            row.update(corner_count=len(corners) if found else 0)
            if not found:
                row["reasons"].append("complete_chessboard_not_found")
            else:
                if reference_size is None:
                    reference_size = size
                elif size != reference_size:
                    row["reasons"].append("mixed_image_resolution")
                corners = np.asarray(corners, np.float32).reshape(-1, 1, 2)
                points = corners.reshape(-1, 2)
                horizontal, _ = cv2.estimateChessboardSharpness(gray, pattern, corners, .8, False)
                vertical, _ = cv2.estimateChessboardSharpness(gray, pattern, corners, .8, True)
                edges = [float(horizontal[0]), float(vertical[0])]
                contrast = min(float(horizontal[2]-horizontal[1]), float(vertical[2]-vertical[1]))
                coverage = float(cv2.contourArea(cv2.convexHull(points)) / (width*height) * 100)
                row.update(edge_horizontal_px=edges[0], edge_vertical_px=edges[1], contrast=contrast,
                           inner_hull_percent=coverage)
                if not np.isfinite(edges).all() or any(value <= 0 or value > args.max_edge_px for value in edges):
                    row["reasons"].append("blurred_edges")
                if not np.isfinite(contrast) or contrast < args.min_contrast:
                    row["reasons"].append("insufficient_black_white_contrast")
                if coverage < args.min_coverage_percent:
                    row["reasons"].append("board_too_small")
                transform, _ = cv2.findHomography(grid, points, 0)
                if transform is None:
                    row["reasons"].append("invalid_board_geometry")
                else:
                    border = np.array([[[-1., -1.], [args.columns, -1.],
                                        [args.columns, args.rows], [-1., args.rows]]], np.float32)
                    border = cv2.perspectiveTransform(border, transform)[0]
                    inside = (np.isfinite(border).all() and np.all(border[:, 0] >= 0)
                              and np.all(border[:, 0] < width) and np.all(border[:, 1] >= 0)
                              and np.all(border[:, 1] < height))
                    if not inside:
                        row["reasons"].append("board_boundary_out_of_frame")
        row["accepted"] = not row["reasons"]
        records.append(row)
        print(f"{'ACCEPT' if row['accepted'] else 'QUARANTINE'} {path.name}: {', '.join(row['reasons']) or 'initial prototype candidate'}", flush=True)

    report = dict(applied=False, pattern_inner_corners=list(pattern), source_directory=str(source_root),
                  accepted_directory=str(accepted_root), quarantine_directory=str(quarantine_root),
                  max_edge_px=args.max_edge_px, min_contrast=args.min_contrast,
                  min_coverage_percent=args.min_coverage_percent,
                  accepted_count=sum(row["accepted"] for row in records),
                  rejected_count=sum(not row["accepted"] for row in records), images=records)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    if args.apply:
        # Preflight every snapshot before moving any original or copying the selected set.
        for path, row in zip(paths, records):
            if hashlib.sha256(path.read_bytes()).hexdigest() != row["sha256"]:
                raise SystemExit("an input changed during review; nothing was moved")
        args.accepted.mkdir(parents=True, exist_ok=True)
        args.quarantine.mkdir(parents=True, exist_ok=True)
        for path, row in zip(paths, records):
            target = (args.accepted if row["accepted"] else args.quarantine) / path.name
            if row["accepted"]:
                shutil.copy2(path, target)
            else:
                path.rename(target)
        (args.accepted / "selection.json").write_text(json.dumps(
            dict(pattern_inner_corners=list(pattern), review_report=str(args.report.resolve()),
                 images=[row for row in records if row["accepted"]]), indent=2) + "\n")
        (args.quarantine / "reasons.json").write_text(json.dumps(
            [row for row in records if not row["accepted"]], indent=2) + "\n")
        report["applied"] = True
        args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(f"Accepted {report['accepted_count']}; quarantined {report['rejected_count']}; applied={report['applied']}")
    print("Selection is for initial PnP debugging; real calibration and distance checks are still required.")


if __name__ == "__main__":
    main()
