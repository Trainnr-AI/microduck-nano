#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Time ONNX models on the board's CPU with ONNX Runtime. Run ON the board (see README.md).

    python bench_onnx.py models/ --json results/onnx.json [--seconds 6] [--threads 1,2,4]

For every model and thread count: random input, 5 warm-up runs, then runs for --seconds; reports the median and
95th-percentile time per inference, the rate, the process's peak memory and the CPU temperature afterwards.
"""
import argparse
import glob
import json
import os
import resource
import time

import numpy as np
import onnxruntime as ort


def temp_c():
    t = []
    for f in glob.glob("/sys/class/thermal/thermal_zone*/temp"):
        try:
            t.append(int(open(f).read()) / 1000)
        except OSError:
            pass
    return max(t) if t else None


def bench(path, threads, seconds):
    so = ort.SessionOptions()
    so.intra_op_num_threads = threads
    so.inter_op_num_threads = 1
    t0 = time.perf_counter()
    s = ort.InferenceSession(path, so, providers=["CPUExecutionProvider"])
    load = time.perf_counter() - t0
    feeds = {}
    for i in s.get_inputs():
        shape = [d if isinstance(d, int) else 1 for d in i.shape]
        feeds[i.name] = np.random.default_rng(0).random(shape, dtype=np.float32)
    for _ in range(5):
        s.run(None, feeds)
    ts, end = [], time.perf_counter() + seconds
    while time.perf_counter() < end or len(ts) < 10:
        a = time.perf_counter()
        s.run(None, feeds)
        ts.append(time.perf_counter() - a)
    ts = np.array(ts) * 1000
    return {"model": os.path.basename(path), "threads": threads, "runs": len(ts), "load_s": round(load, 2),
            "median_ms": round(float(np.median(ts)), 3), "p95_ms": round(float(np.percentile(ts, 95)), 3),
            "max_ms": round(float(ts.max()), 3), "hz": round(1000 / float(np.median(ts)), 1),
            "peak_rss_mb": round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024, 0), "temp_c": temp_c()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("models")
    ap.add_argument("--json")
    ap.add_argument("--seconds", type=float, default=6)
    ap.add_argument("--threads", default="1,2,4")
    ap.add_argument("--only", default="")
    a = ap.parse_args()
    rows = []
    print(f"onnxruntime {ort.__version__}, start temperature {temp_c()} C")
    for path in sorted(glob.glob(os.path.join(a.models, "*.onnx"))):
        if a.only and a.only not in path:
            continue
        for th in [int(x) for x in a.threads.split(",")]:
            r = bench(path, th, a.seconds)
            rows.append(r)
            print(f"{r['model']:28} threads {th}: {r['median_ms']:9.3f} ms median, {r['p95_ms']:9.3f} p95, {r['hz']:8.1f} /s, "
                  f"{r['peak_rss_mb']:.0f} MB, {r['temp_c']} C", flush=True)
            if a.json:
                json.dump(rows, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
