#!/usr/bin/env python3
"""Test flow-rate errors on the real executable; optional idle Webots TCP fixture."""
import argparse
import os
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=Path)
    parser.add_argument('--webots-url', help='Idle matching Webots world for range cases')
    args = parser.parse_args()
    binary = args.binary.resolve()
    cases = [(value, 'expected a finite positive number')
             for value in ('', '0', '-1', 'nan', 'inf', '1e309', '1e-400', 'abc', '0.2junk')]
    if args.webots_url:
        cases += [(value, 'platform timing interval out of range')
                  for value in ('1e-10', '1e-20')]
    for value, expected in cases:
        env = dict(os.environ, WEBOTS_SIM_FLOW_RATE=value)
        if args.webots_url:
            env['WEBOTS_CONTROLLER_URL'] = args.webots_url
        result = subprocess.run([str(binary)], env=env, capture_output=True,
                                text=True, timeout=15)
        assert result.returncode == 2 and expected in result.stderr, (
            value, result.returncode, result.stdout, result.stderr)
    print(f'PASS: {len(cases)} invalid flow-rate startup cases')


if __name__ == '__main__':
    main()
