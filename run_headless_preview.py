#!/usr/bin/env python3
import argparse
import os
import queue
import subprocess
import sys
import threading
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Run Webots preview world with extern rm_auto_aim controller.')
    repo_default = Path(__file__).resolve().parent
    parser.add_argument('--repo', type=Path, default=repo_default)
    parser.add_argument('--world', type=Path,
                        default=Path('webots/worlds/auto_aim_test_field_target_vehicle_camera_preview.wbt'))
    parser.add_argument('--controller', type=Path, default=Path('build/rm_auto_aim'))
    parser.add_argument('--sim-flow-rate', type=float, default=0.1,
                        help='Simulation time flow rate relative to real time. Default: 0.1')
    parser.add_argument('--runtime-sec', type=float, default=0.0,
                        help='Real-time duration before stopping. 0 means run until interrupted.')
    parser.add_argument('--port', type=int, default=1235)
    parser.add_argument('--freq-probe', action='store_true')
    parser.add_argument('--run-root', type=Path, default=Path('.vscode-runs'))
    return parser.parse_args()


def utc_stamp() -> str:
    return time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())


def ensure_symlink(run_dir: Path, repo: Path) -> None:
    tracker_dir = run_dir / 'Modules' / 'ArmorTracker'
    tracker_dir.mkdir(parents=True, exist_ok=True)
    target = repo / 'Modules' / 'ArmorTracker' / 'table.bin'
    link = tracker_dir / 'table.bin'
    if link.exists() or link.is_symlink():
        link.unlink()
    link.symlink_to(target)


def terminate(proc: subprocess.Popen | None) -> int | None:
    if proc is None:
        return None
    if proc.poll() is None:
        proc.terminate()
        try:
            return proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
            return proc.wait(timeout=5)
    return proc.returncode


def wait_for_exit(proc: subprocess.Popen | None, timeout: float) -> int | None:
    if proc is None:
        return None
    if proc.poll() is not None:
        return proc.returncode
    try:
        return proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        return None


def pump(name: str, proc: subprocess.Popen, out_q: queue.Queue[tuple[str, str]]) -> None:
    if proc.stdout is None:
        return
    for line in proc.stdout:
        out_q.put((name, line))
    out_q.put((name, '__EOF__'))


def env_snapshot(env: dict[str, str], keys: list[str]) -> str:
    return ' '.join(f'{key}={env.get(key, "")}' for key in keys)


def main() -> int:
    args = parse_args()
    repo = args.repo.resolve()
    world = (repo / args.world).resolve() if not args.world.is_absolute() else args.world.resolve()
    controller = (repo / args.controller).resolve() if not args.controller.is_absolute() else args.controller.resolve()

    if not world.is_file():
        print(f'world not found: {world}', file=sys.stderr)
        return 2
    if not controller.is_file():
        print(f'controller not found: {controller}', file=sys.stderr)
        return 2

    run_root = args.run_root if args.run_root.is_absolute() else repo / args.run_root
    run_dir = run_root / f'webots_preview_{utc_stamp()}'
    run_dir.mkdir(parents=True, exist_ok=True)
    ensure_symlink(run_dir, repo)

    launcher_log = run_dir / '00_launcher.log'
    webots_log = run_dir / '10_webots.log'
    controller_log = run_dir / '20_controller.log'
    summary = run_dir / '99_summary.txt'

    def log(msg: str) -> None:
        line = f'[{time.strftime("%F %T")}] {msg}'
        print(line, flush=True)
        with launcher_log.open('a', encoding='utf-8') as f:
            f.write(line + '\n')

    webots_env = os.environ.copy()
    webots_env['WEBOTS_HOME'] = '/usr/local/webots'
    webots_cmd = [
        'xvfb-run', '-a', '/usr/local/bin/webots',
        f'--port={args.port}', '--stdout', '--stderr', '--batch', '--mode=fast',
        '--extern-urls', str(world),
    ]
    log('run_dir=' + str(run_dir))
    log('starting webots: ' + ' '.join(webots_cmd))
    webots = subprocess.Popen(
        webots_cmd,
        cwd=str(repo),
        env=webots_env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    controller_proc = None
    controller_started = False
    controller_url = ''
    start_real = time.time()
    out_q: queue.Queue[tuple[str, str]] = queue.Queue()
    threads: list[threading.Thread] = []
    threads.append(threading.Thread(target=pump, args=('webots', webots, out_q), daemon=True))
    threads[-1].start()

    try:
        with webots_log.open('w', encoding='utf-8') as wf, controller_log.open('w', encoding='utf-8') as cf:
            webots_eof = False
            controller_eof = False
            while True:
                if args.runtime_sec > 0 and time.time() - start_real >= args.runtime_sec and controller_started:
                    break
                if webots_eof and (controller_proc is None or controller_eof):
                    break
                try:
                    name, payload = out_q.get(timeout=0.1)
                except queue.Empty:
                    continue

                if payload == '__EOF__':
                    if name == 'webots':
                        webots_eof = True
                    elif name == 'controller':
                        controller_eof = True
                    continue

                if name == 'webots':
                    wf.write(payload)
                    wf.flush()
                    print(payload, end='')
                    s = payload.rstrip('\n')
                    if (not controller_started) and (s.startswith('ipc://') or s.startswith('tcp://')):
                        controller_started = True
                        controller_url = s
                        ctrl_env = os.environ.copy()
                        ctrl_env['WEBOTS_HOME'] = '/usr/local/webots'
                        ctrl_env['WEBOTS_CONTROLLER_URL'] = controller_url
                        ctrl_env['WEBOTS_SIM_FLOW_RATE'] = f'{args.sim_flow_rate:g}'
                        ctrl_env['QT_QPA_PLATFORM'] = 'offscreen'
                        if args.freq_probe:
                            ctrl_env['XR_FREQ_PROBE'] = '1'
                        ld = ctrl_env.get('LD_LIBRARY_PATH', '')
                        ctrl_env['LD_LIBRARY_PATH'] = '/usr/local/webots/lib/controller' + (':' + ld if ld else '')
                        ctrl_cmd = ['stdbuf', '-oL', '-eL', str(controller)]
                        log('starting controller: ' + ' '.join(ctrl_cmd) + f' url={controller_url}')
                        log('controller env: ' + env_snapshot(ctrl_env, [
                            'XR_TRACKER_SP_ENABLE_OUTPUT_MEAS_ANCHOR',
                            'XR_SOLVER_USE_TARGET_DZ',
                            'XR_WEBOTS_PITCH_SLEW_STEP_RAD',
                            'XR_TRACKER_TRUTH_COMPARE',
                            'XR_TRACKER_TRUTH_COMPARE_PATH',
                            'XR_TRACKER_TRUTH_COMPARE_MAX_FRAMES',
                            'XR_TRACKER_VIDEO_RECORDER',
                            'XR_TRACKER_VIDEO_PATH',
                            'XR_DETECTOR_TRUTH_COMPARE',
                            'XR_DETECTOR_TRUTH_COMPARE_PATH',
                            'XR_DETECTOR_VIDEO_RECORDER',
                            'XR_DETECTOR_VIDEO_PATH',
                        ]))
                        controller_proc = subprocess.Popen(
                            ctrl_cmd,
                            cwd=str(run_dir),
                            env=ctrl_env,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            text=True,
                            bufsize=1,
                        )
                        threads.append(threading.Thread(target=pump, args=('controller', controller_proc, out_q), daemon=True))
                        threads[-1].start()
                else:
                    cf.write(payload)
                    cf.flush()
                    print('controller: ' + payload, end='')
    except KeyboardInterrupt:
        log('keyboard interrupt, stopping')
    finally:
        webots_rc = terminate(webots)
        controller_rc = wait_for_exit(controller_proc, timeout=5)
        if controller_rc is None:
            controller_rc = terminate(controller_proc)
        summary.write_text(
            '\n'.join([
                'status=PASS' if controller_started else 'status=FAIL',
                f'run_dir={run_dir}',
                f'sim_flow_rate={args.sim_flow_rate:g}',
                f'controller_started={1 if controller_started else 0}',
                f'controller_url={controller_url}',
                f'webots_rc={webots_rc}',
                f'controller_rc={controller_rc}',
            ]) + '\n',
            encoding='utf-8',
        )
        log('summary written to ' + str(summary))
    return 0 if controller_started else 1


if __name__ == '__main__':
    raise SystemExit(main())
