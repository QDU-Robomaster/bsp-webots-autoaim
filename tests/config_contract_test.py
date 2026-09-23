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
MODULES = {module['module'].split('/')[-1]: module for module in CONFIG['modules']}
RUN_CONFIG = (REPO / 'User/run_config.hpp').read_text(encoding='utf-8')
NS = 'AutoAimRunConfig::Webots::'


def args(name):
    """Return one module's ordered named constructor arguments as a mapping."""
    return {key: value for item in MODULES[name]['args'] for key, value in item.items()}


def constant(name):
    """Return the initializer text of one run_config.hpp constant."""
    return re.search(r'\b%s = (.*);' % name, RUN_CONFIG).group(1)


class ConfigContractTest(unittest.TestCase):
    def test_frame_calibration_and_world(self):
        self.assertEqual(constant('MainFrameLayout'),
                         '{.width = 800, .height = 600, .step = 2400, '
                         '.encoding = CameraTypes::Encoding::BGR8}')
        calibration = constant('MainCameraCalibration')
        self.assertIn('.native_width = 800, .native_height = 600', calibration)
        self.assertIn('.distortion_coefficients = {0.0, 0.0, 0.0, 0.0, 0.0}', calibration)
        matrix = [float(v) for v in re.search(r'\.camera_matrix = \{([^}]*)\}', calibration)
                  .group(1).split(',')]
        world = (REPO / 'webots/worlds/auto_aim_test_field_target_vehicle_camera_preview.wbt').read_text()
        fov = float(re.search(r'fieldOfView\s+([0-9.]+)', world).group(1))
        expected_focal = 800.0 / (2 * math.tan(fov / 2))
        self.assertAlmostEqual(matrix[0], expected_focal, places=4)
        self.assertAlmostEqual(matrix[4], expected_focal, places=4)
        for name in ('WebotsCamera', 'CameraFrameSync', 'ArmorDetector', 'ArmorTracker', 'Aimer'):
            self.assertEqual(MODULES[name]['template_args'], [NS + 'MainFrameLayout'])
        for name in ('WebotsCamera', 'Aimer'):
            self.assertEqual(args(name)['calibration'], NS + 'MainCameraCalibration')

    def test_explicit_backend_and_trigger(self):
        detector = args('ArmorDetector')['cfg']
        self.assertEqual(detector['network']['model'], 'ArmorDetectorModel::OPENVINO_640X512')
        self.assertEqual(detector['network']['logit_threshold'], 0.619)
        camera = args('WebotsCamera')['runtime']
        self.assertEqual(camera['fps'], 100)
        self.assertEqual(camera['trigger_period_us'], 20000)
        self.assertEqual(args('CameraSync')['camera_pin'], MODULES['WebotsCamera']['id'])
        self.assertEqual(args('CameraSync')['param']['trigger_period_us'], 20000)
        sync = args('CameraFrameSync')['runtime']  # positional RuntimeParam
        self.assertEqual(sync[0], 'CameraFrameSyncMode::TRIGGER')
        self.assertEqual(sync[2], '"libxr_def_domain"')
        self.assertNotIn('number_refine', detector)

    def test_referee_and_launcher_configuration(self):
        aim = args('Aimer')['cfg']
        self.assertEqual(aim['referee_topic'], '"robot_game_ref"')
        self.assertEqual(aim['default_bullet_speed'], 23.0)
        self.assertEqual(args('WebotsReferee')['param']['bullet_speed'], 23.0)
        self.assertEqual(args('WebotsFireNotify')['param']['bullet_speed'], 23.0)
        dependencies = yaml.safe_load((REPO / 'Modules/modules.yaml').read_text())['modules']
        for dependency in ('xrobot-org/DurationStatistics', 'QDU-Robomaster/Referee', 'QDU-Robomaster/CMD'):
            self.assertIn(dependency + '@same-or-dev', dependencies)
        for path in ('User/main.cpp', 'Modules/QDU-Robomaster/WebotsGimbal/WebotsGimbal.hpp',
                     'Modules/QDU-Robomaster/WebotsFireNotify/WebotsFireNotify.hpp',
                     'Modules/QDU-Robomaster/WebotsReferee/WebotsReferee.hpp'):
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
            preview = args(name)['cfg']['preview']
            self.assertTrue(preview['enabled'], name)
            self.assertEqual(preview['output_mode'], '"web"', name)
            self.assertEqual(preview['web_port'], 8080, name)
            self.assertEqual(preview['web_stream_name'], '"%s"' % stream, name)

    def test_generated_headers_match(self):
        with tempfile.TemporaryDirectory(prefix='webots-codegen-check-') as temporary:
            output = Path(temporary) / 'xrobot_main.hpp'
            result = subprocess.run([sys.executable, '-m', 'xrobot.GenerateMain', '--config',
                                     'User/xrobot.yaml', '--output', str(output),
                                     '--register-source', 'User/main.cpp', '--lock', 'xrobot.lock'],
                                    cwd=REPO, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, check=False)
            self.assertEqual(result.returncode, 0, result.stdout)
            self.assertEqual(output.read_text(encoding='utf-8'),
                             (REPO / 'User/xrobot_main.hpp').read_text(encoding='utf-8'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
