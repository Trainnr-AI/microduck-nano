#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Build the ONNX test models for the UNO Q benchmarks. Run on a PC, not on the board.

    uv run --with onnx --with numpy --with onnxruntime --with ultralytics --with onnxslim python make_models.py out/

Writes:
  policy_small.onnx   MLP 64 -> 256 -> 128 -> 64 -> 14    (a small walking policy)
  policy_large.onnx   MLP 64 -> 512 -> 256 -> 128 -> 14   (the size mjlab trains by default)
  policy_hist.onnx    MLP 640 -> 1024 -> 512 -> 256 -> 14 (10 steps of history)
  policy_nano.onnx    MLP 61 -> 512 -> 256 -> 128 -> 14   (the Nano's observation size; for `duckd policy-time`)
  yolo11n_<size>.onnx       Ultralytics YOLO11n detector, float32, at 160 / 224 / 320 / 640 px
  yolo11n_<size>_int8.onnx  the same, statically quantized to 8 bit (calibrated on RANDOM images:
                            good for timing only, the detections are meaningless)
  yolo11n-pose_320.onnx, yolo11n-seg_320.onnx, yolo11n-cls_224.onnx

The YOLO weights are downloaded by the ultralytics package and are AGPL-3.0: they are used here only
to measure speed and are not redistributed in this repository. Policy weights are random.
"""
import sys
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper


def mlp(path, sizes, seed=0):
    rng = np.random.default_rng(seed)
    nodes, inits, x = [], [], "obs"
    for i, (a, b) in enumerate(zip(sizes[:-1], sizes[1:])):
        inits.append(numpy_helper.from_array((rng.standard_normal((b, a)) / np.sqrt(a)).astype(np.float32), f"w{i}"))
        inits.append(numpy_helper.from_array(np.zeros(b, np.float32), f"b{i}"))
        last = i == len(sizes) - 2
        nodes.append(helper.make_node("Gemm", [x, f"w{i}", f"b{i}"], ["action" if last else f"h{i}"], transB=1))
        if not last:
            nodes.append(helper.make_node("Elu", [f"h{i}"], [f"a{i}"]))
            x = f"a{i}"
    g = helper.make_graph(nodes, "policy", [helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, sizes[0]])],
                          [helper.make_tensor_value_info("action", TensorProto.FLOAT, [1, sizes[-1]])], inits)
    m = helper.make_model(g, opset_imports=[helper.make_opsetid("", 17)])
    m.ir_version = 9
    onnx.checker.check_model(m)
    onnx.save(m, path)


def quantize(src, dst, size):
    from onnxruntime.quantization import CalibrationDataReader, QuantFormat, QuantType, quantize_static
    from onnxruntime.quantization.shape_inference import quant_pre_process

    class Reader(CalibrationDataReader):
        def __init__(self):
            rng = np.random.default_rng(0)
            self.it = iter([{"images": rng.random((1, 3, size, size), dtype=np.float32)} for _ in range(8)])

        def get_next(self):
            return next(self.it, None)

    pre = str(dst) + ".pre"
    quant_pre_process(str(src), pre)
    quantize_static(pre, str(dst), Reader(), quant_format=QuantFormat.QDQ, activation_type=QuantType.QUInt8,
                    weight_type=QuantType.QInt8, per_channel=True)
    Path(pre).unlink()


def main():
    out = Path(sys.argv[1] if len(sys.argv) > 1 else "out")
    out.mkdir(parents=True, exist_ok=True)
    mlp(out / "policy_small.onnx", [64, 256, 128, 64, 14])
    mlp(out / "policy_large.onnx", [64, 512, 256, 128, 14])
    mlp(out / "policy_hist.onnx", [640, 1024, 512, 256, 14])
    # The Nano's own shape: 61 observations (as Microduck's policies), the size mjlab trains by default.
    # runtime/tests/data/tiny_policy.json describes the observation layout; copy it beside this file to run it.
    mlp(out / "policy_nano.onnx", [61, 512, 256, 128, 14])
    from ultralytics import YOLO
    work = out / "_work"
    work.mkdir(exist_ok=True)
    jobs = [("yolo11n", s) for s in (160, 224, 320, 640)] + [("yolo11n-pose", 320), ("yolo11n-seg", 320), ("yolo11n-cls", 224)]
    for name, size in jobs:
        pt = work / f"{name}.pt"
        f = YOLO(str(pt)).export(format="onnx", imgsz=size, opset=17, simplify=True, dynamic=False)
        dst = out / f"{name}_{size}.onnx"
        Path(f).replace(dst)
        if name == "yolo11n":
            quantize(dst, out / f"{name}_{size}_int8.onnx", size)
    for f in sorted(out.glob("*.onnx")):
        print(f"{f.name:28} {f.stat().st_size / 1e6:6.1f} MB")


if __name__ == "__main__":
    main()
