#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Write the tiny ONNX policy the runtime's tests load. Not a trained policy.

    uv run --with onnx --with numpy python make_fixture.py

tiny_policy/policy.onnx (its manifest is tiny_policy/deploy.json, trainnr's format): obs[1,61] -> action[1,14], two layers with an ELU between, weights chosen so the tests can
compute the expected output by hand:
  hidden = ELU(obs[0:14] * 1.0 + 0)          (the first 14 observation values pass through; negatives are squashed)
  action = hidden * 0.5 + 0.1
"""
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

w0 = np.zeros((14, 61), np.float32)
w0[np.arange(14), np.arange(14)] = 1.0
w1 = (np.eye(14) * 0.5).astype(np.float32)
nodes = [
    helper.make_node("Gemm", ["obs", "w0", "b0"], ["h"], transB=1),
    helper.make_node("Elu", ["h"], ["a"]),
    helper.make_node("Gemm", ["a", "w1", "b1"], ["action"], transB=1),
]
inits = [numpy_helper.from_array(w0, "w0"), numpy_helper.from_array(np.zeros(14, np.float32), "b0"),
         numpy_helper.from_array(w1, "w1"), numpy_helper.from_array(np.full(14, 0.1, np.float32), "b1")]
graph = helper.make_graph(nodes, "tiny_policy",
                          [helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, 61])],
                          [helper.make_tensor_value_info("action", TensorProto.FLOAT, [1, 14])], inits)
model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
model.ir_version = 9
onnx.checker.check_model(model)
out = Path(__file__).with_name("tiny_policy") / "policy.onnx"
onnx.save(model, out)
print(out.name, out.stat().st_size, "bytes")
