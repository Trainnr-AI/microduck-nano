#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Time ncnn models on the board's CPU and on its GPU through Vulkan. Run ON the board (see README.md).

    python bench_ncnn.py ncnn_models/ --json results/ncnn.json [--seconds 6]

ncnn_models/ holds one folder per model, named <name>_<size>, each with model.ncnn.param and model.ncnn.bin
(Ultralytics' ncnn export). Every model runs on the CPU with 4 threads, then on each Vulkan device ncnn lists
(the real GPU and Mesa's software renderer), with random input. The first GPU run includes shader compilation and is
reported separately as "first_ms".
"""
import argparse
import glob
import json
import os
import time

import ncnn
import numpy as np


def temp_c():
    t = []
    for f in glob.glob("/sys/class/thermal/thermal_zone*/temp"):
        try:
            t.append(int(open(f).read()) / 1000)
        except OSError:
            pass
    return max(t) if t else None


def bench(folder, device, seconds, threads=4, fp16=True):
    size = int(folder.rstrip("/").rsplit("_", 1)[1])
    net = ncnn.Net()
    net.opt.num_threads = threads
    net.opt.use_vulkan_compute = device >= 0
    net.opt.use_fp16_packed = net.opt.use_fp16_storage = net.opt.use_fp16_arithmetic = fp16
    if device >= 0:
        net.set_vulkan_device(device)
    t0 = time.perf_counter()
    net.load_param(os.path.join(folder, "model.ncnn.param"))
    net.load_model(os.path.join(folder, "model.ncnn.bin"))
    load = time.perf_counter() - t0
    x = np.random.default_rng(0).random((3, size, size), dtype=np.float32)
    outs = [n for n in net.output_names()]

    def once():
        ex = net.create_extractor()
        ex.input(net.input_names()[0], ncnn.Mat(x))
        a = time.perf_counter()
        for o in outs:
            ret, m = ex.extract(o)
            if ret != 0:
                raise RuntimeError(f"extract {o} failed: {ret}")
            np.array(m)
        return (time.perf_counter() - a) * 1000

    first = once()
    for _ in range(3):
        once()
    ts, end = [], time.perf_counter() + seconds
    while time.perf_counter() < end or len(ts) < 5:
        ts.append(once())
    ts = np.array(ts)
    return {"load_s": round(load, 2), "first_ms": round(first, 1), "median_ms": round(float(np.median(ts)), 2),
            "p95_ms": round(float(np.percentile(ts, 95)), 2), "hz": round(1000 / float(np.median(ts)), 1), "runs": len(ts)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("models")
    ap.add_argument("--json")
    ap.add_argument("--seconds", type=float, default=6)
    ap.add_argument("--only", default="")
    a = ap.parse_args()
    devs = [(-1, "cpu x4")] + [(i, ncnn.get_gpu_info(i).device_name()) for i in range(ncnn.get_gpu_count())]
    print("ncnn", ncnn.__version__, "devices:", devs, "start temperature", temp_c())
    rows = []
    for folder in sorted(glob.glob(os.path.join(a.models, "*_*"))):
        if a.only and a.only not in folder:
            continue
        for dev, name in devs:
            if "llvmpipe" in name and int(folder.rsplit("_", 1)[1]) > 224:
                continue                      # the software renderer is only a reference point; skip the slow sizes
            row = {"model": os.path.basename(folder), "device": name}
            try:
                row.update(bench(folder, dev, a.seconds))
                print(f"{row['model']:18} {name:28} {row['median_ms']:9.2f} ms median, {row['p95_ms']:9.2f} p95, {row['hz']:6.1f} /s, "
                      f"first run {row['first_ms']:.0f} ms, load {row['load_s']} s, {temp_c()} C", flush=True)
            except Exception as e:           # a driver failure on one model must not stop the rest
                row["error"] = repr(e)
                print(f"{row['model']:18} {name:28} FAILED {e!r}", flush=True)
            row["temp_c"] = temp_c()
            rows.append(row)
            if a.json:
                json.dump(rows, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
