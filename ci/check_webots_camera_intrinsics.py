#!/usr/bin/env python3
"""Validate Webots camera geometry against the generated CameraInfo YAML."""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
YAML_PATH = ROOT / "User" / "xrobot.yaml"
WORLD_PATH = ROOT / "webots/worlds/auto_aim_test_field_target_vehicle_camera_preview.wbt"


def fail(message: str) -> None:
    print(f"camera intrinsic check failed: {message}", file=sys.stderr)
    raise SystemExit(1)


def read_number_list_after(lines: list[str], key: str) -> list[float]:
    for index, line in enumerate(lines):
        if re.match(rf"^\s*{re.escape(key)}:\s*$", line):
            values: list[float] = []
            for item in lines[index + 1 :]:
                stripped = item.strip()
                if not stripped:
                    continue
                if not stripped.startswith("- "):
                    break
                values.append(float(stripped[2:]))
            return values
    fail(f"missing YAML key {key}")


def read_scalar_after(lines: list[str], key: str) -> float:
    for line in lines:
        match = re.match(rf"^\s*{re.escape(key)}:\s*([-+0-9.eE]+)\s*$", line)
        if match:
            return float(match.group(1))
    fail(f"missing YAML scalar {key}")


def extract_camera_block(world_text: str) -> str:
    marker = "DEF camera Camera"
    start = world_text.find(marker)
    if start < 0:
        fail("missing DEF camera Camera block")
    brace_start = world_text.find("{", start)
    if brace_start < 0:
        fail("camera block has no opening brace")

    depth = 0
    for index in range(brace_start, len(world_text)):
        char = world_text[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return world_text[brace_start + 1 : index]
    fail("camera block has no closing brace")


def read_world_scalar(camera_block: str, key: str) -> float:
    match = re.search(rf"^\s*{re.escape(key)}\s+([-+0-9.eE]+)\s*$", camera_block, re.M)
    if not match:
        fail(f"missing world camera field {key}")
    return float(match.group(1))


def assert_close(name: str, actual: float, expected: float, tolerance: float = 1e-6) -> None:
    if abs(actual - expected) > tolerance:
        fail(f"{name} mismatch: actual={actual:.12g} expected={expected:.12g}")


def main() -> int:
    yaml_lines = YAML_PATH.read_text(encoding="utf-8").splitlines()
    world_text = WORLD_PATH.read_text(encoding="utf-8")
    camera_block = extract_camera_block(world_text)

    yaml_width = int(read_scalar_after(yaml_lines, "width"))
    yaml_height = int(read_scalar_after(yaml_lines, "height"))
    camera_matrix = read_number_list_after(yaml_lines, "camera_matrix")
    distortion = read_number_list_after(yaml_lines, "distortion_coefficients")

    if len(camera_matrix) != 9:
        fail(f"camera_matrix has {len(camera_matrix)} entries, expected 9")

    world_fov = read_world_scalar(camera_block, "fieldOfView")
    world_width = int(read_world_scalar(camera_block, "width"))
    world_height = int(read_world_scalar(camera_block, "height"))

    if yaml_width != world_width or yaml_height != world_height:
        fail(
            f"resolution mismatch: yaml={yaml_width}x{yaml_height} "
            f"world={world_width}x{world_height}"
        )

    expected_focal = world_width / (2.0 * math.tan(world_fov / 2.0))
    assert_close("fx", camera_matrix[0], expected_focal, 1e-6)
    assert_close("fy", camera_matrix[4], expected_focal, 1e-6)
    assert_close("cx", camera_matrix[2], world_width / 2.0, 1e-9)
    assert_close("cy", camera_matrix[5], world_height / 2.0, 1e-9)

    has_lens = re.search(r"^\s*lens\s+Lens\s*{", camera_block, re.M) is not None
    zero_distortion = all(abs(value) <= 1e-12 for value in distortion)
    if has_lens and zero_distortion:
        fail("world camera has Lens distortion but YAML distortion_coefficients are zero")
    if not has_lens and not zero_distortion:
        fail("world camera is pinhole but YAML distortion_coefficients are nonzero")

    print(
        "camera intrinsic check passed: "
        f"{world_width}x{world_height}, fov={world_fov}, "
        f"fx={camera_matrix[0]:.12g}, distortion={'lens' if has_lens else 'zero'}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
