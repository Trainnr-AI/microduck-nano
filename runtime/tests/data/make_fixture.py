#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Write the tiny policy the runtime's tests load: a network and its manifest. Not a trained policy.

    uv run --with onnx --with numpy python make_fixture.py

tiny_policy/deploy.json is shaped as trainnr's `trainnr-deploy/1` (trainnr docs/87) with every required key, and
tiny_policy/policy.onnx takes its 51 observation values to 14 actions in two layers with an ELU between, weights
chosen so the tests can compute the expected output by hand:
  hidden = ELU(obs[0:14])          (the joint positions pass through; negatives are squashed)
  action = hidden * 0.5 + 0.1
"""
import json
import math
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

JOINTS = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle", "neck_pitch", "head_pitch",
          "head_yaw", "head_roll", "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle"]
N = len(JOINTS)
TERMS = [("joint_pos", N, "joint_pos_rel"), ("joint_vel", N, "joint_vel_rel"), ("actions", N, "last_action"),
         ("projected_gravity", 3, "projected_gravity"), ("base_ang_vel", 3, "sensor robot/imu_ang_vel"),
         ("command", 3, "command twist")]
WIDTH = sum(w for _, w, _ in TERMS)
out = Path(__file__).with_name("tiny_policy")

manifest = {
    "schema": "trainnr-deploy/1",
    "policy": "tiny test policy (not trained)",
    "run": "none (written by make_fixture.py)",
    "robot": "microduck-nano",
    "control": {"physics_timestep_s": 0.005, "decimation": 4, "control_hz": 50, "episode_length_s": 20.0},
    "joints": {"policy_order": JOINTS, "ctrl_order": JOINTS, "action_to_ctrl": list(range(N)), "default_pos": [0.0] * N,
               "stiffness": [None] * N, "damping": [None] * N, "effort_limit": [None] * N},
    "action": {"kind": "joint position target = default_pos + scale * action", "scale": [math.radians(10)] * N,
               "offset": [0.0] * N, "clip": [-1.0, 1.0]},
    "observations": [{"name": name, "width": w, "source": source, "params": {}, "scale": 1.0, "clip": None,
                      "history_length": 0} for name, w, source in TERMS],
    "onnx": {"file": "policy.onnx", "input": "obs", "output": "action", "input_width": WIDTH, "output_width": N,
             "normalization": "folded into the graph"},
    "scene": {"file": "scene.xml"},
    "commands": {"twist": {"lin_vel_x": [-0.4, 0.4], "lin_vel_y": [-0.3, 0.3], "ang_vel_z": [-1.0, 1.0]}},
}
(out / "deploy.json").write_text(json.dumps(manifest, indent=1) + "\n")

w0 = np.zeros((N, WIDTH), np.float32)
w0[np.arange(N), np.arange(N)] = 1.0
w1 = (np.eye(N) * 0.5).astype(np.float32)
nodes = [
    helper.make_node("Gemm", ["obs", "w0", "b0"], ["h"], transB=1),
    helper.make_node("Elu", ["h"], ["a"]),
    helper.make_node("Gemm", ["a", "w1", "b1"], ["action"], transB=1),
]
inits = [numpy_helper.from_array(w0, "w0"), numpy_helper.from_array(np.zeros(N, np.float32), "b0"),
         numpy_helper.from_array(w1, "w1"), numpy_helper.from_array(np.full(N, 0.1, np.float32), "b1")]
graph = helper.make_graph(nodes, "tiny_policy",
                          [helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, WIDTH])],
                          [helper.make_tensor_value_info("action", TensorProto.FLOAT, [1, N])], inits)
model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
model.ir_version = 9
onnx.checker.check_model(model)
onnx.save(model, out / "policy.onnx")
print("deploy.json and policy.onnx:", WIDTH, "->", N)
