#!/usr/bin/env python3
"""Launcher outcomes on fixtures: the fake controller writes the run summary the real one
writes, so the judgement is tested without Webots or inference."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

LAUNCHER = Path(__file__).resolve().parents[1] / 'run_headless_preview.py'

# 各模式下假控制器写出的摘要 / Summary the fake controller writes in each mode.
SUMMARIES = {
    'healthy': '{"synced": 30, "detected": 30, "armors": 40, "tracked": 30, "tracking": 25, '
               '"aimed": 30, "control": 25, "fire": 5, "shots": 2, "errors": 0}',
    'stalled_stage': '{"synced": 30, "detected": 30, "armors": 40, "tracked": 0, "tracking": 0, '
                     '"aimed": 0, "control": 0, "fire": 0, "shots": 0, "errors": 0}',
    'no_target': '{"synced": 30, "detected": 30, "armors": 0, "tracked": 30, "tracking": 0, '
                 '"aimed": 30, "control": 0, "fire": 0, "shots": 0, "errors": 0}',
    'errors': '{"synced": 30, "detected": 30, "armors": 40, "tracked": 30, "tracking": 25, '
              '"aimed": 30, "control": 25, "fire": 5, "shots": 2, "errors": 1}',
    'crash': '{"synced": 3, "detected": 3, "armors": 4, "tracked": 3, "tracking": 0, '
             '"aimed": 3, "control": 0, "fire": 0, "shots": 0, "errors": 0}',
}


class LauncherTest(unittest.TestCase):
    def run_fixture(self, mode):
        with tempfile.TemporaryDirectory(prefix='webots-launcher-test-') as temporary:
            root = Path(temporary)
            home, bins = root / 'webots', root / 'bin'
            home.mkdir()
            bins.mkdir()
            world = root / 'scene.wbt'
            world.write_text('# launcher fixture, not a rendered simulation\n')
            wrapper = bins / 'xvfb-run'
            wrapper.write_text('#!' + sys.executable + '\nimport os,sys\nos.execvp(sys.argv[2],sys.argv[2:])\n')
            simulator = home / 'webots'
            simulator.write_text('#!' + sys.executable + '\n'
                                 'import os,time\n'
                                 'if os.environ["LAUNCHER_FIXTURE"] != "no_url": print("ipc://fixture/self",flush=True)\n'
                                 'time.sleep(20)\n')
            controller = root / 'controller'
            controller.write_text('#!' + sys.executable + '\n'
                                  'import json,os,time\n'
                                  'mode=os.environ["LAUNCHER_FIXTURE"]\n'
                                  f'summaries={SUMMARIES!r}\n'
                                  'if mode in summaries:\n'
                                  '    open(os.environ["XR_RUN_SUMMARY"],"w").write(summaries[mode])\n'
                                  'if mode == "crash": raise SystemExit(7)\n'
                                  'time.sleep(20)\n')
            for path in (wrapper, simulator, controller):
                path.chmod(0o755)
            env = os.environ.copy()
            env.update(WEBOTS_HOME=str(home), LAUNCHER_FIXTURE=mode,
                       PATH=str(bins) + os.pathsep + env['PATH'])
            result = subprocess.run(
                [sys.executable, str(LAUNCHER), '--repo', str(root), '--world', str(world),
                 '--controller', str(controller), '--run-root', str(root / 'runs'),
                 '--runtime-sec', '0.6', '--startup-timeout-sec', '1.2'],
                env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=10)
            summaries = list((root / 'runs').glob('*/99_summary.txt'))
            self.assertEqual(len(summaries), 1, result.stdout)
            fields = dict(line.split('=', 1) for line in summaries[0].read_text().splitlines())
            return result.returncode, fields

    def test_healthy_run_passes(self):
        code, fields = self.run_fixture('healthy')
        self.assertEqual(code, 0)
        self.assertEqual(fields['status'], 'PASS')
        self.assertEqual(fields['summary_detected'], '30')

    def test_stalled_stage_fails(self):
        code, fields = self.run_fixture('stalled_stage')
        self.assertNotEqual(code, 0)
        self.assertEqual(fields['reason'], 'stage_without_frames')

    def test_no_detection_or_tracking_fails(self):
        code, fields = self.run_fixture('no_target')
        self.assertNotEqual(code, 0)
        self.assertEqual(fields['reason'], 'no_detection_or_tracking')

    def test_logged_error_fails(self):
        code, fields = self.run_fixture('errors')
        self.assertNotEqual(code, 0)
        self.assertEqual(fields['reason'], 'errors_logged')

    def test_controller_crash_fails(self):
        code, fields = self.run_fixture('crash')
        self.assertNotEqual(code, 0)
        self.assertIn('controller_exited_early:7', fields['reason'])

    def test_missing_summary_fails(self):
        code, fields = self.run_fixture('silent')
        self.assertNotEqual(code, 0)
        self.assertEqual(fields['reason'], 'no_run_summary')

    def test_connection_timeout_fails(self):
        code, fields = self.run_fixture('no_url')
        self.assertNotEqual(code, 0)
        self.assertEqual(fields['reason'], 'controller_connection_timeout')

    def test_nonfinite_flow_rate_is_rejected(self):
        result = subprocess.run([sys.executable, str(LAUNCHER), '--sim-flow-rate', 'nan'],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        self.assertEqual(result.returncode, 2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
