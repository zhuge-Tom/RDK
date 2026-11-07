#!/usr/bin/env python3
"""Record a static, user-positioned single-target PnP comparison over local HTTP."""
import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import shutil
import statistics
import time
import urllib.request

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--url', default='http://127.0.0.1:8876', help='browser HTTP base URL')
p.add_argument('--calibration', type=Path, help='local copy of the active calibration for the snapshot')
p.add_argument('--reference-origin', default='lens front to armor center',
               help='physical tape endpoints; defaults to the center-distance protocol')
p.add_argument('--reference-m', type=float, required=True)
p.add_argument('--uncertainty-m', type=float)
p.add_argument('--approximate-reference', action='store_true',
               help='nominal reference only; do not invent an uncertainty bound')
p.add_argument('--samples', type=int, default=50)
p.add_argument('--interval', type=float, default=.2)
a = p.parse_args()
numbers = [a.reference_m, a.interval] + ([] if a.uncertainty_m is None else [a.uncertainty_m])
if (not all(math.isfinite(v) for v in numbers)
        or a.reference_m <= 0 or (a.uncertainty_m is not None and a.uncertainty_m < 0)
        or a.samples < 10 or a.interval < .05):
    p.error('invalid reference, uncertainty, count or polling interval')
if a.approximate_reference and a.uncertainty_m is not None:
    p.error('use a nominal approximate reference or an explicit bound, not both')
if a.calibration is not None and not a.calibration.is_file():
    p.error('--calibration must name an existing local configuration')
base_url = a.url.rstrip('/')
if not base_url.startswith(('http://', 'https://')):
    p.error('--url must be an HTTP(S) browser URL')
a.output.mkdir(parents=True, exist_ok=False)
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

def status():
    with opener.open(base_url + '/status', timeout=2) as r:
        return json.load(r)

ready = 0
for _ in range(80):
    try:
        s = status()
        ready = ready + 1 if (s.get('stream_alive') and s.get('blue_armors') == 1
                  and len(s.get('geometry', {}).get('poses', [])) == 1) else 0
        if ready >= 3:
            break
    except OSError:
        ready = 0
    time.sleep(.15)
else:
    raise SystemExit('single target not ready; no measurements fabricated')

rows, raw = [], []
started = datetime.now().isoformat()
start = time.monotonic()
for i in range(a.samples):
    row = dict(sample=i+1, elapsed_s=round(time.monotonic()-start, 5), valid_pose=False)
    try:
        s = status()
        raw.append(s)
        row.update(stream_alive=s.get('stream_alive'), blue_armors=s.get('blue_armors'))
        poses = s.get('geometry', {}).get('poses', [])
        if s.get('stream_alive') and s.get('blue_armors') == 1 and len(poses) == 1:
            pose = poses[0]
            if all(math.isfinite(v) for v in pose['position_m'] + [pose['distance_m'], pose['reprojection_error_px']]):
                row.update(valid_pose=True, distance_m=pose['distance_m'], z_m=pose['position_m'][2],
                           reprojection_error_px=pose['reprojection_error_px'])
                corners = s.get('corners', [])
                if len(corners) == 4:
                    row.update(center_x_px=sum(c[0] for c in corners)/4,
                               center_y_px=sum(c[1] for c in corners)/4)
    except (OSError, ValueError, KeyError) as error:
        row['error'] = str(error)
    rows.append(row)
    delay = start + (i+1)*a.interval - time.monotonic()
    if delay > 0:
        time.sleep(delay)

valid = [r for r in rows if r['valid_pose']]
meta = dict(started_local=started, duration_s=round(time.monotonic()-start, 3),
            reference_distance_m=a.reference_m, reference_uncertainty_m=a.uncertainty_m,
            reference_is_approximate=a.approximate_reference or a.uncertainty_m is None,
            reference_origin=a.reference_origin, camera_optical_center_offset_m=None,
            user_confirmed_new_fixed_placement=True, metric_accuracy_verified=False,
            sample_count=len(rows), valid_pose_samples=len(valid), poll_interval_s=a.interval,
            sampling_note='HTTP polls, not all camera frames; reference and optical-center offset limits apply')
if valid:
    values = [r['distance_m'] for r in valid]
    mean = statistics.mean(values)
    meta.update(mean_distance_m=mean, median_distance_m=statistics.median(values),
                std_distance_m=statistics.pstdev(values), min_distance_m=min(values),
                max_distance_m=max(values), mean_z_m=statistics.mean(r['z_m'] for r in valid),
                median_reprojection_px=statistics.median(r['reprojection_error_px'] for r in valid),
                signed_difference_to_nominal_m=mean-a.reference_m,
                signed_difference_to_reference_interval_m=(None if a.uncertainty_m is None else
                    [mean-a.reference_m-a.uncertainty_m, mean-a.reference_m+a.uncertainty_m]))
    c = [r for r in valid if 'center_x_px' in r]
    if c:
        meta.update(center_x_span_px=max(r['center_x_px'] for r in c)-min(r['center_x_px'] for r in c),
                    center_y_span_px=max(r['center_y_px'] for r in c)-min(r['center_y_px'] for r in c))
(a.output/'samples.json').write_text(json.dumps(dict(metadata=meta, samples=rows, raw_status=raw), indent=2)+'\n')
(a.output/'summary.json').write_text(json.dumps(meta, indent=2)+'\n')
with (a.output/'samples.csv').open('w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=sorted(set().union(*(r.keys() for r in rows))), lineterminator='\n')
    w.writeheader()
    w.writerows(rows)
config_root = Path(__file__).resolve().parents[1] / 'config'
snapshot = config_root / 'blue_vision_ros.yaml'
if snapshot.is_file():
    shutil.copy2(snapshot, a.output / snapshot.name)
if a.calibration is not None:
    shutil.copy2(a.calibration, a.output / 'pose_camera.snapshot.yml')
print(json.dumps(meta, indent=2))
