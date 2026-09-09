#!/usr/bin/env python3
"""Check the formal Webots deployment contract and generated-header reproducibility."""
import math
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest
import yaml

REPO = Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load((REPO / 'User/xrobot.yaml').read_text(encoding='utf-8'))
MODULES = {module['name']: module for module in CONFIG['modules']}


class ConfigContractTest(unittest.TestCase):
    def test_frame_calibration_and_world(self):
        layout = CONFIG['constexprs']['MainFrameLayout']['value']
        calibration = CONFIG['constexprs']['MainCameraCalibration']['value']
        self.assertEqual(layout, dict(width=800, height=600, step=2400,
                                      encoding='CameraTypes::Encoding::BGR8'))
        self.assertEqual((calibration['native_width'], calibration['native_height']), (800, 600))
        self.assertEqual(calibration['distortion_coefficients'], [0.0] * 5)
        world = (REPO / 'webots/worlds/auto_aim_test_field_target_vehicle_camera_preview.wbt').read_text()
        fov = float(re.search(r'fieldOfView\s+([0-9.]+)', world).group(1))
        expected_focal = 800.0 / (2 * math.tan(fov / 2))
        self.assertAlmostEqual(calibration['camera_matrix'][0], expected_focal, places=4)
        self.assertAlmostEqual(calibration['camera_matrix'][4], expected_focal, places=4)
        for name in ('WebotsCamera', 'CameraFrameSync', 'ArmorDetector', 'ArmorTracker', 'Aimer'):
            self.assertEqual(MODULES[name]['template_args'], {'Layout': {'constexpr': 'MainFrameLayout'}})
        for name in ('WebotsCamera', 'Aimer'):
            self.assertEqual(MODULES[name]['constructor_args']['calibration'],
                             {'constexpr': 'MainCameraCalibration'})

    def test_explicit_backend_and_trigger(self):
        detector = MODULES['ArmorDetector']['constructor_args']['cfg']
        self.assertEqual(detector['network']['model'], {'expr': 'ArmorDetectorModel::OPENVINO_640X512'})
        self.assertEqual(detector['network']['logit_threshold'], 0.619)
        camera = MODULES['WebotsCamera']['constructor_args']['runtime']
        self.assertEqual(camera['fps'], 100)
        self.assertEqual(camera['trigger_period_us'], 20000)
        self.assertEqual(MODULES['CameraSync']['constructor_args']['trigger_period_us'], 20000)
        sync = MODULES['CameraFrameSync']['constructor_args']['runtime']
        self.assertEqual(sync['mode'], {'expr': 'CameraFrameSyncMode::TRIGGER'})
        self.assertEqual(sync['host_topic_domain_name'], 'libxr_def_domain')
        self.assertNotIn('sync_probe_div', sync)
        self.assertNotIn('number_refine', detector)

    def test_referee_and_launcher_configuration(self):
        aim = MODULES['Aimer']['constructor_args']['cfg']
        self.assertEqual(aim['referee_topic'], 'robot_game_ref')
        self.assertEqual(aim['default_bullet_speed'], 23.0)
        self.assertEqual(MODULES['WebotsReferee']['constructor_args']['bullet_speed'], 23.0)
        self.assertEqual(MODULES['WebotsFireNotify']['constructor_args']['bullet_speed'], 23.0)
        dependencies = yaml.safe_load((REPO / 'Modules/modules.yaml').read_text())['modules']
        for dependency in ('xrobot-org/DurationStatistics', 'qdu-future/Referee', 'qdu-future/CMD'):
            self.assertIn(dependency, dependencies)
        for path in ('User/main.cpp', 'Modules/WebotsGimbal/WebotsGimbal.hpp',
                     'Modules/WebotsFireNotify/WebotsFireNotify.hpp',
                     'Modules/WebotsReferee/WebotsReferee.hpp'):
            self.assertIsNone(re.search(r'LibXR::RawData\s*&', (REPO / path).read_text(encoding='utf-8')), path)

    def test_missing_openvino_is_rejected(self):
        with tempfile.TemporaryDirectory(prefix='webots-no-openvino-') as temporary:
            result = subprocess.run(['cmake', '-S', str(REPO), '-B', temporary,
                                     '-DCMAKE_DISABLE_FIND_PACKAGE_OpenVINO=ON'],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, check=False, timeout=45)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('OpenVINO', result.stdout)
            self.assertIn('REQUIRED', result.stdout)

    def test_three_web_previews(self):
        expected = {'ArmorDetector': 'armor_detector',
                    'ArmorTracker': 'armor_tracker', 'Aimer': 'aimer_preview'}
        for name, stream in expected.items():
            preview = MODULES[name]['constructor_args']['cfg']['preview']
            self.assertTrue(preview['enabled'], name)
            self.assertEqual(preview['output_mode'], 'web', name)
            self.assertEqual(preview['web_port'], 8080, name)
            self.assertEqual(preview['web_stream_name'], stream, name)

    def test_generated_headers_match(self):
        with tempfile.TemporaryDirectory(prefix='webots-codegen-check-') as temporary:
            output = Path(temporary) / 'xrobot_main.hpp'
            result = subprocess.run([sys.executable, '-m', 'xrobot.GenerateMain', '--config',
                                     'User/xrobot.yaml', '--output', str(output)], cwd=REPO,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
            self.assertEqual(result.returncode, 0, result.stdout)
            for name in ('xrobot_main.hpp', 'xrobot_constexpr.hpp'):
                self.assertEqual((Path(temporary) / name).read_text(encoding='utf-8'),
                                 (REPO / 'User' / name).read_text(encoding='utf-8'), name)


if __name__ == '__main__':
    unittest.main(verbosity=2)
