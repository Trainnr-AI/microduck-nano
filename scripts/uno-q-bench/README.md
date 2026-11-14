# UNO Q compute benchmarks

Scripts that measure what the Arduino UNO Q can run. Results and how to reproduce them:
[docs/project/uno-q-compute.md](../../docs/project/uno-q-compute.md).

| File | Runs on | What |
|---|---|---|
| `make_models.py` | PC | Builds the ONNX test models (policy networks, YOLO11n variants) |
| `bench_onnx.py` | board | Times ONNX models on the CPU with ONNX Runtime |
| `bench_ncnn.py` | board | Times ncnn models on the CPU and on the GPU through Vulkan |
| `bench_llm.sh` | board | Times small language models with llama.cpp |
| `results/` | | Raw numbers from 2026-11-14 |
