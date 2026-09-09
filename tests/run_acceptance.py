#!/usr/bin/env python3
"""Run the actual Webots BSP with read-only subscribers, then check recorded payloads."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import statistics
import subprocess
import sys
import time

WORLD = Path('webots/worlds/auto_aim_test_field_target_vehicle_camera_preview.wbt')


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding='utf-8') as stream:
        result = list(csv.DictReader(stream, delimiter='\t'))
    require(all(None not in row and all(v is not None for v in row.values()) for row in result),
            f'incomplete evidence row: {path}')
    return result


def finite(row: dict[str, str], keys: tuple[str, ...]) -> bool:
    return all(math.isfinite(float(row[key])) for key in keys)


def quantiles(values: list[float]) -> dict[str, float]:
    ordered = sorted(values)
    return {'min': ordered[0], 'median': statistics.median(ordered),
            'p95': ordered[min(len(ordered) - 1, math.ceil(len(ordered) * 0.95) - 1)],
            'max': ordered[-1]}


def verify(case: Path, mode: str) -> dict:
    summaries = list((case / 'launcher').glob('*/99_summary.txt'))
    require(len(summaries) == 1, 'expected one launcher summary')
    summary = dict(line.split('=', 1) for line in summaries[0].read_text().splitlines())
    require(summary['status'] == 'PASS' and summary['runtime_errors'] == '0',
            f'launcher did not complete cleanly: {summary}')
    require(summary['controller_rc_before_stop'] == 'None', 'controller exited before requested shutdown')
    records = case / 'records'
    frame_rows = rows(records / 'frames.tsv')
    stage_rows = {stage: [r for r in frame_rows if r['stage'] == stage]
                  for stage in ('sync', 'detector', 'tracker')}
    maps = {}
    for stage, data in stage_rows.items():
        require(len(data) >= 30, f'{stage}: fewer than 30 observed frames')
        sequence = [int(row['sequence']) for row in data]
        require(all(a < b for a, b in zip(sequence, sequence[1:])), f'{stage}: non-monotonic sequence')
        require(all(row['valid'] == '1' for row in data), f'{stage}: invalid ownership/geometry')
        require(all(row['image_ts'] == row['imu_ts'] for row in data), f'{stage}: trigger timestamp mismatch')
        timestamp = [int(row['imu_ts']) for row in data]
        require(all(a < b for a, b in zip(timestamp, timestamp[1:])), f'{stage}: timestamp rollback')
        maps[stage] = {row['sequence']: row for row in data}
    for stage in ('detector', 'tracker'):
        for seq, row in maps[stage].items():
            require(seq in maps['sync'], f'{stage}: frame not present at input')
            original = maps['sync'][seq]
            require(all(row[key] == original[key] for key in ('image_id', 'image_ts', 'imu_ts')),
                    f'{stage}: shared image identity/timestamps changed at frame {seq}')
    # Source construction starts acquisition before inference/tracker subscriptions exist.
    # Treat that prefix separately; require no losses after each stage begins publishing.
    startup_prefix = {}
    for stage in ('detector', 'tracker'):
        first = int(stage_rows[stage][0]['sequence'])
        last = int(stage_rows[stage][-1]['sequence'])
        expected = {seq for seq in maps['sync'] if first <= int(seq) <= last}
        require(set(maps[stage]) == expected, f'{stage}: missing frames after subscription startup')
        startup_prefix[stage] = sum(int(seq) < first for seq in maps['sync'])
    tail_frames = sum(int(seq) > int(stage_rows['tracker'][-1]['sequence']) for seq in maps['sync'])
    require(tail_frames <= 2, 'too many in-flight frames at bounded process shutdown')
    detections = rows(records / 'detections.tsv')
    tracking = rows(records / 'tracking.tsv')
    commands = rows(records / 'commands.tsv')
    firing = rows(records / 'firing.tsv')
    referee = rows(records / 'referee.tsv')
    require(len(tracking) == len(stage_rows['tracker']), 'tracker payload evidence missing')
    require(abs(len(commands) - len(tracking)) <= 1 and abs(len(firing) - len(commands)) <= 1,
            'Aimer publication count does not follow Tracker')
    require(all(row['finite'] == '1' for row in tracking + commands), 'non-finite tracker/control state')
    require(all(finite(row, ('x', 'y', 'z', 'v_yaw')) for row in tracking), 'non-finite target payload')
    require(all(finite(row, ('roll', 'yaw', 'roll_vel', 'yaw_vel', 'roll_acc', 'yaw_acc'))
                for row in commands), 'non-finite command payload')
    require(referee and all(int(row['size']) > 31 and row['robot_id'] == '7' and
                           row['heat_limit'] == '240' and row['cooling'] == '40' for row in referee),
            'current Referee payload did not arrive intact')
    tracking_count = sum(row['tracking'] == '1' for row in tracking)
    nonzero_commands = sum(abs(float(row['roll'])) + abs(float(row['yaw'])) > 1e-6 for row in commands)
    fire_count = sum(row['fire'] == '1' for row in firing)
    result = {
        'mode': mode, 'status': 'PASS', 'stage_frames': {s: len(v) for s, v in stage_rows.items()},
        'detections': len(detections), 'tracking_frames': tracking_count,
        'command_count': len(commands), 'nonzero_commands': nonzero_commands,
        'fire_requests': fire_count, 'referee_messages': len(referee),
        'referee_payload_bytes': int(referee[0]['size']),
        'shared_frame_identity_timestamps_geometry_order': 'PASS',
        'source_frames_before_first_stage_output': startup_prefix,
        'source_frames_in_flight_at_shutdown': tail_frames,
        'shutdown': 'bounded process termination; at most two in-flight frames; not a destructor/drain test',
        'sim_time_span_s': (int(stage_rows['tracker'][-1]['imu_ts']) -
                            int(stage_rows['tracker'][0]['imu_ts'])) / 1e6,
    }
    if mode == 'target':
        require(len(detections) >= 20 and tracking_count >= 20 and nonzero_commands >= 20,
                'target did not traverse detector/PnP/tracker/aimer')
        require(all(row['quad_valid'] == '1' for row in detections), 'invalid or off-image corner quadrilateral')
        require(all(row['pnp_valid'] == '1' and finite(row, ('reprojection_px', 'tx', 'ty', 'tz', 'confidence'))
                    for row in detections), 'invalid PnP/confidence payload')
        require(all(0.0 <= float(row['confidence']) <= 1.0 and 0.5 < float(row['tz']) < 8.0 and
                    0.0 <= float(row['reprojection_px']) < 10.0 for row in detections),
                'PnP inconsistent with the approximately 3 m test rig or excessive reprojection residual')
        correct_labels = sum(row['number'] == '2' and row['color'] == '1' for row in detections)
        require(correct_labels >= 20, 'did not recognize the blue digit-3 simulated armor')
        result.update({
            'pnp_valid': sum(row['pnp_valid'] == '1' for row in detections),
            'blue_three_detections': correct_labels,
            'reprojection_px': quantiles([float(row['reprojection_px']) for row in detections]),
            'camera_depth_m': quantiles([float(row['tz']) for row in detections]),
            'corners': 'finite, in bounds, convex, canonical winding',
            'pose_accuracy_claim': 'coarse range and reprojection sanity only; not world-ground-truth pose error',
        })
        require((records / 'target-camera.png').is_file() and (records / 'target-corners.png').is_file(),
                'actual rendered target images missing')
    else:
        require(not detections and tracking_count == 0 and nonzero_commands == 0 and fire_count == 0,
                'empty scene produced a detection, target or active command')
    return result


def run_case(repo: Path, controller: Path, root: Path, mode: str, runtime: float, port: int) -> dict:
    case = root / mode
    case.mkdir(parents=True, exist_ok=False)
    world = repo / WORLD
    original_hash = hashlib.sha256(world.read_bytes()).hexdigest()
    if mode == 'empty':
        # Change only the target's presence in an isolated fixture, not production assets.
        fixture = case / 'fixture' / 'webots'
        shutil.copytree(repo / 'webots', fixture,
                        ignore=shutil.ignore_patterns('__pycache__', '*.log'))
        world = fixture / 'worlds' / WORLD.name
        text = world.read_text(encoding='utf-8')
        original = 'translation 0 1.21215e-06 -0.28'
        require(text.count(original) == 1, 'canonical target pose changed; update fixture deliberately')
        require(text.count('controller "forced_target_simple"') == 1, 'target controller not unique')
        text = text.replace(original, 'translation 1000 0 -0.28', 1)
        text = text.replace('controller "forced_target_simple"', 'controller "<none>"', 1)
        world.write_text(text, encoding='utf-8')
    (case / 'scenario.json').write_text(json.dumps({
        'mode': mode, 'source_world': str(repo / WORLD), 'source_world_sha256': original_hash,
        'world': str(world), 'world_sha256': hashlib.sha256(world.read_bytes()).hexdigest(),
        'controller': str(controller), 'controller_sha256': hashlib.sha256(controller.read_bytes()).hexdigest(),
        'device': 'CPU', 'requested_sim_flow_rate': 0.1,
        'physical_device_access': False,
    }, indent=2) + '\n')
    env = os.environ.copy()
    env.update(XR_WEBOTS_ACCEPTANCE_DIR=str(case / 'records'), XR_ARMOR_OPENVINO_DEVICE='CPU',
               LIBGL_ALWAYS_SOFTWARE='1', QTWEBENGINE_DISABLE_SANDBOX='1')
    command = [sys.executable, str(repo / 'run_headless_preview.py'), '--repo', str(repo),
               '--world', str(world), '--controller', str(controller), '--runtime-sec', str(runtime),
               '--sim-flow-rate', '0.1', '--port', str(port), '--run-root', str(case / 'launcher')]
    with (case / 'console.log').open('w', encoding='utf-8') as log:
        completed = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                   timeout=runtime + 90, check=False)
    require(completed.returncode == 0, f'{mode} launcher failed ({completed.returncode}), see {case / "console.log"}')
    result = verify(case, mode)
    (case / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller', type=Path, required=True)
    parser.add_argument('--run-root', type=Path, required=True)
    parser.add_argument('--case', choices=('target', 'empty', 'both'), default='both')
    parser.add_argument('--runtime-sec', type=float, default=40)
    parser.add_argument('--port', type=int, default=1250)
    args = parser.parse_args()
    require(math.isfinite(args.runtime_sec) and args.runtime_sec > 0, 'runtime must be positive and finite')
    repo = Path(__file__).resolve().parents[1]
    root = args.run_root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    controller = args.controller.resolve()
    try:
        modes = ('target', 'empty') if args.case == 'both' else (args.case,)
        results = [run_case(repo, controller, root, mode, args.runtime_sec, args.port + i)
                   for i, mode in enumerate(modes)]
        (root / ('result-' + args.case + '.json')).write_text(json.dumps(results, indent=2) + '\n')
    except Exception as error:
        (root / ('failure-' + args.case + '.txt')).write_text(str(error) + '\n')
        print(f'FAIL: {error}', file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
