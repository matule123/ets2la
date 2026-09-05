"""Read-only inputs -> local steering evidence; never starts the game/Engine.

Video mode decodes EVERY frame and measures EVERY adjacent frame pair. Optical
flow is a screen-space observation, not a calibrated tyre angle or lane CTE.
Camera changes/occlusion must be labelled separately during visual review.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import statistics


def log_rows(path, start, end):
    rows = []
    with Path(path).open(encoding="utf-8", errors="replace") as stream:
        for line_number, line in enumerate(stream, 1):
            if not start <= line[:19] <= end or "autopilot: active=" not in line:
                continue
            row = dict(re.findall(r"(\w+)=([^\s]+)", line))
            for key, value in tuple(row.items()):
                number = re.fullmatch(r"([-+\d.eE]+)(?:deg|kmh|/s2|/s|m)?", value)
                if number:
                    try:
                        row[key] = float(number[1])
                    except ValueError:
                        pass
                elif value in ("True", "False"):
                    row[key] = value == "True"
            row.update(line_number=line_number, timestamp=line[:23], original=line.strip())
            row["time_s"] = datetime.fromisoformat(line[:23].replace(",", ".")).timestamp()
            rows.append(row)
    return rows


def identify(rows):
    """Independent quasi-steady Frenet estimate, with explicitly limited scope.

    k_vehicle = k_lane*cos(h)/(1-k_lane*LaneMatchCTE) - dh/dt/v.
    Assumes local lane curvature ~ preview curvature only in nearly constant
    same-LaneId windows. No estimate is made at a turn entry or LaneId boundary.
    SDK wheelbase is absent in these old logs: report angle per 3.8 m AND gain.
    """
    estimates = []
    for a, b, c in zip(rows, rows[1:], rows[2:]):
        if not all(x.get("active") and x.get("nav") for x in (a, b, c)):
            continue
        dt = c["time_s"] - a["time_s"]
        v, k, u = b["speed"] / 3.6, b.get("preview_k", 0), b.get("game_steer", 0)
        h = math.radians(b["lane_heading"])
        dh = math.radians(c["lane_heading"] - a["lane_heading"])
        ks = [x.get("preview_k", 0) for x in (a, b, c)]
        if not (1.5 < dt < 2.8 and v > 5 and abs(u) > .025 and abs(k) > .003
                and abs(h) < .04 and abs(dh) < .025
                and abs(c["lane_cte"] - a["lane_cte"]) < .09
                and max(ks) - min(ks) < .0015
                and a["LaneId"] == b["LaneId"] == c["LaneId"]):
            continue
        actual = k * math.cos(h) / (1 - k * b["lane_cte"]) - dh / dt / v
        estimates.append(dict(timestamp=b["timestamp"], line=b["line_number"],
                              curvature_per_m=actual, command=u,
                              full_angle_rad_at_3_8m=math.atan(3.8 * actual) / u))
    return estimates


def video(path, output):
    import cv2
    import numpy as np

    output.mkdir(parents=True, exist_ok=True)
    capture = cv2.VideoCapture(str(path))
    previous = None
    index = 0
    # Screen coordinates only. Cockpit steering-wheel crop in this 1080p clip;
    # recorded in CSV so overhead/camera-cut pairs are never mislabelled as yaw.
    timestamps = []
    contact = []
    with (output / "adjacent-frames.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(("frame", "pts_s", "dt_s", "wheel_rotation_deg", "flow_residual_px", "features", "whole_frame_change"))
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            pts = capture.get(cv2.CAP_PROP_POS_MSEC) / 1000
            small = cv2.resize(frame, (640, 360))
            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
            rotation = residual = change = 0.0
            count = 0
            if previous is not None:
                mask = np.zeros_like(gray)
                # Wheel centre/lower rim; use available height (no padding).
                cv2.ellipse(mask, (330, 270), (78, 65), 0, 0, 360, 255, -1)
                points = cv2.goodFeaturesToTrack(previous, 120, .01, 7, mask=mask)
                change = float(np.mean(cv2.absdiff(previous, gray)))
                if points is not None:
                    nxt, status, _ = cv2.calcOpticalFlowPyrLK(previous, gray, points, None)
                    good = status.reshape(-1) == 1
                    count = int(good.sum())
                    if count >= 6:
                        transform, inliers = cv2.estimateAffinePartial2D(points[good], nxt[good], method=cv2.RANSAC)
                        if transform is not None:
                            rotation = math.degrees(math.atan2(transform[1, 0], transform[0, 0]))
                            predicted = points[good].reshape(-1, 2) @ transform[:, :2].T + transform[:, 2]
                            residual = float(np.median(np.linalg.norm(predicted - nxt[good].reshape(-1, 2), axis=1)))
            writer.writerow((index, pts, pts - timestamps[-1] if timestamps else 0,
                             rotation, residual, count, change))
            # An index, not a substitute for adjacent-frame measurement. Dense
            # strips for detected events can be regenerated with --frames.
            if index % 160 == 0:
                cv2.putText(small, f"frame {index} / {pts:.3f}s", (8, 22), 0, .6, (0, 255, 255), 1)
                contact.append(small)
            previous = gray
            timestamps.append(pts)
            index += 1
    capture.release()
    for page in range(0, len(contact), 12):
        tiles = contact[page:page + 12]
        tiles += [np.zeros((360, 640, 3), dtype=np.uint8)] * (12 - len(tiles))
        sheet = np.vstack([np.hstack(tiles[i:i + 3]) for i in range(0, 12, 3)])
        cv2.imwrite(str(output / f"index-{page // 12}.jpg"), sheet)
    metadata = dict(video=str(path), frames=index, pairs=max(0, index - 1),
                    first_pts_s=timestamps[0], last_pts_s=timestamps[-1],
                    median_dt_s=statistics.median(b-a for a,b in zip(timestamps,timestamps[1:])),
                    mask_640x360=dict(centre=[330,270],ellipse_radii=[78,65]),
                    warning="Screen-space optical flow, not physical wheel angle; camera cuts/overhead views invalid.")
    (output / "video.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(json.dumps(metadata))


def frame_strip(path, output, first, count):
    """Export every adjacent frame in a short event, with exact media PTS."""
    import cv2
    import numpy as np
    capture = cv2.VideoCapture(str(path))
    # CAP_PROP_POS_FRAMES seeks by nominal FPS in this VFR MP4 and can land
    # ~0.3 s away. Count decoded frames from the beginning, exactly as the
    # adjacent-frame audit does, so index and presentation timestamp agree.
    for _ in range(first):
        if not capture.read()[0]:
            break
    tiles = []
    for index in range(first, first+count):
        ok, frame = capture.read()
        if not ok:
            break
        pts = capture.get(cv2.CAP_PROP_POS_MSEC)/1000
        # Include the wheel and nearby road; the full-frame indices are kept
        # separately so cropped/camera-relative evidence is never called CTE.
        crop = frame[int(frame.shape[0]*.53):int(frame.shape[0]*.98),
                     int(frame.shape[1]*.32):int(frame.shape[1]*.72)]
        tile = cv2.resize(crop, (400,250))
        cv2.putText(tile, f'{index}: {pts:.4f}s', (5,20), 0,.55,(0,255,255),1)
        tiles.append(tile)
    capture.release()
    if not tiles:
        raise ValueError('No video frames in requested range')
    tiles += [np.zeros_like(tiles[0])] * ((-len(tiles)) % 4)
    sheet = np.vstack([np.hstack(tiles[n:n+4]) for n in range(0,len(tiles),4)])
    output.parent.mkdir(parents=True,exist_ok=True)
    if not cv2.imwrite(str(output), sheet):
        raise OSError(f'Cannot export {output}')
    print(str(output.resolve()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path)
    parser.add_argument("--start", default="2026-08-16 11:29:00")
    parser.add_argument("--end", default="2026-08-16 11:51:04")
    parser.add_argument("--video", type=Path)
    parser.add_argument("--frames", type=int, nargs=2, metavar=('FIRST','COUNT'))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    if not args.output.resolve().is_relative_to(root):
        parser.error("Audit outputs must stay in the UltraPilot workspace")
    if args.log:
        rows = log_rows(args.log, args.start, args.end)
        fits = identify(rows)
        data = dict(source=str(args.log), start=args.start, end=args.end,
                    source_sha256=hashlib.file_digest(args.log.open('rb'), 'sha256').hexdigest(),
                    rows=rows, quasi_steady_estimates=fits,
                    limitation="~1 Hz logs; no actual axle angle/pose per tick. Not an exact closed-loop replay.")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
        print(json.dumps(dict(rows=len(rows), estimates=fits)))
    if args.video:
        if args.frames:
            frame_strip(args.video, args.output, *args.frames)
        else:
            video(args.video, args.output)


if __name__ == "__main__":
    main()
