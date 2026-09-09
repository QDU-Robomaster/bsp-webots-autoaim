#!/usr/bin/env python3
"""Regression tests for launcher outcomes; fixtures do not perform vision inference."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

LAUNCHER = Path(__file__).resolve().parents[1] / 'run_headless_preview.py'


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
                                 'if os.environ["LAUNCHER_FIXTURE"] == "forwarded_error": print("controller: E [2] inference failed",flush=True)\n'
                                 'time.sleep(20)\n')
            controller = root / 'controller'
            controller.write_text('#!' + sys.executable + '\n'
                                  'import os,time\n'
                                  'mode=os.environ["LAUNCHER_FIXTURE"]\n'
                                  'if mode in ("detect_begin", "publish_begin", "publish_end"): print("I [1] ArmorDetector trace frame=30 step="+mode,flush=True)\n'
                                  'elif mode != "no_frames": print("I [1] ArmorDetector frame=30 armors=0",flush=True)\n'
                                  'if mode == "crash_after_frame": raise SystemExit(7)\n'
                                  'if mode in ("runtime_error", "delayed_error"): print("E [2] inference failed",flush=True)\n'
                                  'time.sleep(20)\n')
            for path in (wrapper, simulator, controller):
                path.chmod(0o755)
            env = os.environ.copy()
            env.update(WEBOTS_HOME=str(home), LAUNCHER_FIXTURE=mode,
                       PATH=str(bins) + os.pathsep + env['PATH'])
            launch = LAUNCHER
            if mode == 'delayed_error':
                launch = root / 'delayed_reader.py'
                launch.write_text(
                    'import importlib.util,time\n'
                    f'spec=importlib.util.spec_from_file_location("launcher",{str(LAUNCHER)!r})\n'
                    'module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)\n'
                    'def delayed(name,process,output):\n'
                    ' for line in process.stdout:\n'
                    '  if "E [2]" in line:\n'
                    '   while process.poll() is None: time.sleep(0.005)\n'
                    '  output.put((name,line))\n'
                    ' output.put((name,None))\n'
                    'module.pump=delayed\n'
                    'raise SystemExit(module.main())\n')
            result = subprocess.run(
                [sys.executable, str(launch), '--repo', str(root), '--world', str(world),
                 '--controller', str(controller), '--run-root', str(root / 'runs'),
                 '--runtime-sec', '0.6', '--startup-timeout-sec', '1.2'],
                env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=10)
            summaries = list((root / 'runs').glob('*/99_summary.txt'))
            self.assertEqual(len(summaries), 1, result.stdout)
            fields = dict(line.split('=', 1) for line in summaries[0].read_text().splitlines())
            fields['saved_controller_log'] = (summaries[0].parent / '20_controller.log').read_text()
            return result.returncode, fields

    def test_healthy_process_and_frames(self):
        code, fields = self.run_fixture('healthy')
        self.assertEqual(code, 0)
        self.assertEqual(fields['status'], 'PASS')
        self.assertEqual(fields['controller_rc_before_stop'], 'None')

    def test_crash_after_a_frame_is_failure(self):
        code, fields = self.run_fixture('crash_after_frame')
        self.assertNotEqual(code, 0)
        self.assertEqual(fields['status'], 'FAIL')
        self.assertIn('controller_exited_early:7', fields['reason'])

    def test_connected_but_no_pipeline_is_failure(self):
        code, fields = self.run_fixture('no_frames')
        self.assertNotEqual(code, 0)
        self.assertEqual(fields['status'], 'FAIL')

    def test_runtime_error_is_failure(self):
        code, fields = self.run_fixture('runtime_error')
        self.assertNotEqual(code, 0)
        self.assertEqual(fields['runtime_errors'], '1')

    def test_incomplete_traces_are_not_completed_frames(self):
        for mode in ('detect_begin', 'publish_begin'):
            with self.subTest(mode=mode):
                code, fields = self.run_fixture(mode)
                self.assertNotEqual(code, 0)
                self.assertEqual(fields['detector_frames_observed'], '0')

    def test_publish_end_is_a_completed_frame(self):
        code, fields = self.run_fixture('publish_end')
        self.assertEqual(code, 0)
        self.assertEqual(fields['detector_frames_observed'], '30')

    def test_already_emitted_error_delayed_in_reader_is_collected(self):
        code, fields = self.run_fixture('delayed_error')
        self.assertNotEqual(code, 0)
        self.assertEqual(fields['runtime_errors'], '1')
        self.assertIn('E [2] inference failed', fields['saved_controller_log'])

    def test_forwarded_controller_error_is_failure(self):
        code, fields = self.run_fixture('forwarded_error')
        self.assertNotEqual(code, 0)
        self.assertEqual(fields['runtime_errors'], '1')

    def test_connection_timeout_is_failure(self):
        code, fields = self.run_fixture('no_url')
        self.assertNotEqual(code, 0)
        self.assertEqual(fields['reason'], 'controller_connection_timeout')

    def test_nonfinite_flow_rate_is_rejected(self):
        result = subprocess.run([sys.executable, str(LAUNCHER), '--sim-flow-rate', 'nan'],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        self.assertEqual(result.returncode, 2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
