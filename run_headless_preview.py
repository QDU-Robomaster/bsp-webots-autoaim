#!/usr/bin/env python3
"""Launch the Webots BSP headless and judge the run from the controller's run summary.

The controller writes run_summary.json (frames per stage, detections, tracking, aim and
fire frames, error logs) every second; a run passes when every stage received frames,
detections and tracking occurred, and no error was logged."""
import argparse
import json
import math
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import sys
import threading
import time

STAGES = ('synced', 'detected', 'tracked', 'aimed')


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument('--world', type=Path,
                        default=Path('webots/worlds/auto_aim_test_field_target_vehicle_camera_preview.wbt'))
    parser.add_argument('--controller', type=Path, default=Path('build/rm_auto_aim'))
    parser.add_argument('--sim-flow-rate', type=float, default=0.1)
    parser.add_argument('--runtime-sec', type=float, default=0.0,
                        help='Controller wall-clock run duration; 0 runs until interrupted.')
    parser.add_argument('--startup-timeout-sec', type=float, default=60.0)
    parser.add_argument('--port', type=int, default=1235)
    parser.add_argument('--run-root', type=Path, default=Path('.vscode-runs'))
    args = parser.parse_args()
    if (not all(math.isfinite(value) for value in (args.sim_flow_rate, args.runtime_sec, args.startup_timeout_sec))
            or args.sim_flow_rate <= 0 or args.runtime_sec < 0 or args.startup_timeout_sec <= 0):
        parser.error('flow rate/startup timeout must be positive; runtime must be nonnegative')
    return args


def terminate_group(process):
    """Stop the launched process group, including xvfb-run's simulator children."""
    if process is None:
        return None
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    if process.poll() is None:
        process.wait(timeout=5)
    return process.returncode


def pump(name, process, output):
    if process.stdout is not None:
        for line in process.stdout:
            output.put((name, line))
    output.put((name, None))


def read_summary(path, previous):
    """The latest run summary; the previous one while the file is missing or partial."""
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return previous


def judge(summary):
    """(passed, reason) of a finished run from its summary."""
    if not summary:
        return False, 'no_run_summary'
    if not all(summary.get(stage, 0) > 0 for stage in STAGES):
        return False, 'stage_without_frames'
    if summary.get('armors', 0) == 0 or summary.get('tracking', 0) == 0:
        return False, 'no_detection_or_tracking'
    if summary.get('errors', 0):
        return False, 'errors_logged'
    return True, 'duration_reached'


def main():
    args = parse_args()
    repo = args.repo.resolve()
    world = (repo / args.world).resolve()
    controller = (repo / args.controller).resolve()
    for path in (world, controller):
        if not path.is_file():
            print(f'missing file: {path}', file=sys.stderr)
            return 2
    run_dir = (repo / args.run_root).resolve() / time.strftime('webots_preview_%Y%m%dT%H%M%SZ', time.gmtime())
    run_dir.mkdir(parents=True, exist_ok=False)
    summary_path = run_dir / 'run_summary.json'
    environment = os.environ.copy()
    environment.setdefault('WEBOTS_HOME', '/usr/local/webots')
    environment.setdefault('USER', 'xrobot')
    # Webots itself uses Xvfb/XCB even when the controller is headless.
    environment['QT_QPA_PLATFORM'] = 'xcb'
    webots_command = [
        'xvfb-run', '-a', str(Path(environment['WEBOTS_HOME']) / 'webots'),
        f'--port={args.port}', '--stdout', '--stderr', '--batch', '--mode=fast',
        '--extern-urls', str(world),
    ]
    webots = controller_process = None
    controller_url = ''
    controller_started_at = None
    summary = {}
    outcome = 'FAIL'
    reason = 'startup_failed'
    start = time.monotonic()
    output = queue.Queue()
    readers = []
    ansi = re.compile(r'\x1b\[[0-9;]*m')
    with (run_dir / '00_launcher.log').open('w', encoding='utf-8') as launch_log, \
         (run_dir / '10_webots.log').open('w', encoding='utf-8') as webots_log, \
         (run_dir / '20_controller.log').open('w', encoding='utf-8') as controller_log:
        def log(message):
            text = f'[{time.strftime("%F %T")}] {message}'
            print(text, flush=True)
            launch_log.write(text + '\n')
            launch_log.flush()

        def record_line(name, line):
            if line is None:
                return ''
            destination = webots_log if name == 'webots' else controller_log
            destination.write(line)
            destination.flush()
            print(('controller: ' if name == 'controller' else '') + line, end='', flush=True)
            return ansi.sub('', line).strip()

        def start_reader(name, process):
            reader = threading.Thread(target=pump, args=(name, process, output), daemon=True)
            readers.append(reader)
            reader.start()

        try:
            log(f'run_dir={run_dir}')
            log('starting Webots: ' + ' '.join(webots_command))
            webots = subprocess.Popen(webots_command, cwd=repo, env=environment,
                                      stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                      text=True, errors='replace', bufsize=1,
                                      start_new_session=True)
            start_reader('webots', webots)
            while True:
                now = time.monotonic()
                if webots.poll() is not None:
                    reason = f'webots_exited_early:{webots.returncode}'
                    break
                if controller_process is not None and controller_process.poll() is not None:
                    reason = f'controller_exited_early:{controller_process.returncode}'
                    break
                if controller_started_at is None and now - start >= args.startup_timeout_sec:
                    reason = 'controller_connection_timeout'
                    break
                if controller_started_at is not None:
                    elapsed = now - controller_started_at
                    summary = read_summary(summary_path, summary)
                    if summary.get('detected', 0) == 0 and elapsed >= args.startup_timeout_sec:
                        reason = 'pipeline_startup_timeout'
                        break
                    if args.runtime_sec > 0 and elapsed >= args.runtime_sec:
                        reason = 'duration_reached'
                        break
                try:
                    name, line = output.get(timeout=0.1)
                except queue.Empty:
                    continue
                plain = record_line(name, line)
                if name == 'webots' and controller_process is None and plain.startswith(('ipc://', 'tcp://')):
                    controller_url = plain
                    controller_env = environment.copy()
                    controller_env['WEBOTS_CONTROLLER_URL'] = controller_url
                    controller_env['WEBOTS_SIM_FLOW_RATE'] = str(args.sim_flow_rate)
                    controller_env['XR_RUN_SUMMARY'] = str(summary_path)
                    command = ['stdbuf', '-oL', '-eL', str(controller)]
                    log('starting controller: ' + ' '.join(command) + ' url=' + controller_url)
                    # 在仓库根目录运行，模型路径 armor-models/ 相对于它 / Runs from the
                    # repository root, which the model path armor-models/ is relative to.
                    controller_process = subprocess.Popen(
                        command, cwd=repo, env=controller_env,
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                        text=True, errors='replace', bufsize=1, start_new_session=True)
                    controller_started_at = time.monotonic()
                    start_reader('controller', controller_process)
        except KeyboardInterrupt:
            outcome, reason = 'STOPPED', 'user_interrupt'
        except Exception as error:
            reason = f'launcher_exception:{error}'
            log(reason)
        finally:
            controller_before_stop = None if controller_process is None else controller_process.poll()
            webots_before_stop = None if webots is None else webots.poll()
            controller_rc = terminate_group(controller_process)
            webots_rc = terminate_group(webots)
            for reader in readers:
                reader.join(timeout=2)
            while True:
                try:
                    record_line(*output.get_nowait())
                except queue.Empty:
                    break
            summary = read_summary(summary_path, summary)
            if reason == 'duration_reached':
                if controller_before_stop is not None:
                    reason = f'controller_exited_early:{controller_before_stop}'
                elif webots_before_stop is not None:
                    reason = f'webots_exited_early:{webots_before_stop}'
                else:
                    passed, reason = judge(summary)
                    outcome = 'PASS' if passed else 'FAIL'
            fields = {
                'status': outcome, 'reason': reason, 'run_dir': str(run_dir),
                'sim_flow_rate': args.sim_flow_rate,
                'controller_started': int(controller_process is not None),
                'controller_url': controller_url,
                **{f'summary_{key}': value for key, value in summary.items()},
                'controller_rc_before_stop': controller_before_stop,
                'webots_rc_before_stop': webots_before_stop,
                'controller_rc': controller_rc, 'webots_rc': webots_rc,
            }
            (run_dir / '99_summary.txt').write_text(
                ''.join(f'{key}={value}\n' for key, value in fields.items()), encoding='utf-8')
            log(f'status={outcome} reason={reason} summary={json.dumps(summary)}')
    return 0 if outcome == 'PASS' else (130 if outcome == 'STOPPED' else 1)


if __name__ == '__main__':
    raise SystemExit(main())
