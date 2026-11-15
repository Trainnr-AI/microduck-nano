# What the Arduino UNO Q can run

Measurements of the UNO Q's Linux side as the robot's brain: walking policies, vision models and small language
models, on the CPU and on the GPU. Taken 2026-11-14 on one board; treat them as a first reading, not a datasheet.

**Short version:** the CPU is the only useful compute. A walking policy costs almost nothing. Small vision models
run at 10–40 frames a second if they are small (160–320 px) and 8-bit. Language models below about half a billion
parameters are fast enough to talk, but too weak to choose actions reliably; the ones that choose well are too slow.
The GPU is 6–9 times slower than the CPU for vision and crashes with language models.

## The board as tested

| | |
|---|---|
| Board | Arduino UNO Q, 4 GB, bare on a desk, powered over USB-C, no camera attached |
| Processor | 4 × Arm Cortex-A53 at up to 2.0 GHz (reported as Kryo-V2; no dot-product or half-float instructions) |
| Memory | 3.7 GB, about 3.0 GB free with the robot's app running |
| Storage | 9.8 GB, 3.9 GB free |
| GPU | Adreno 702 through Mesa: Vulkan 1.0 (Turnip), OpenCL 3.0 (rusticl) |
| Software | Debian 13, kernel 6.16.7, Arduino App CLI 0.14.1; ONNX Runtime 1.32.0; ncnn 1.0.20261020; llama.cpp b12383 |

All inputs are random arrays of the right shape, so these are speeds, not accuracy. Times are medians over about
5 seconds of repeated runs.

## Walking policies (ONNX Runtime, CPU)

| Policy network | 1 core | 4 cores |
|---|---|---|
| 64 → 256 → 128 → 64 → 14 | 0.14 ms (7,350 /s) | 0.14 ms |
| 64 → 512 → 256 → 128 → 14 | 0.64 ms (1,550 /s) | 0.52 ms |
| 640 → 1024 → 512 → 256 → 14 (10 steps of history) | 3.2 ms (310 /s) | 1.0 ms |

A policy stepped 50 times a second uses under 4 % of one core at the default size. The policy is not the constraint;
the link to the microcontroller is (115200 baud, about 256-byte messages).

## Vision: YOLO11n on the CPU (ONNX Runtime)

| Model | Input | Float, 4 cores | 8-bit, 4 cores | 8-bit, 2 cores |
|---|---|---|---|---|
| Detection | 160 px | 51 ms (19.5 /s) | 26 ms (37.9 /s) | 37 ms (27.0 /s) |
| Detection | 224 px | 87 ms (11.5 /s) | 43 ms (23.3 /s) | 65 ms (15.4 /s) |
| Detection | 320 px | 163 ms (6.1 /s) | 80 ms (12.5 /s) | 126 ms (7.9 /s) |
| Detection | 640 px | 581 ms (1.7 /s) | 275 ms (3.6 /s) | 472 ms (2.1 /s) |
| Classification | 224 px | 41 ms (24.2 /s) | | |
| Pose | 320 px | 185 ms (5.4 /s) | | |
| Segmentation | 320 px | 227 ms (4.4 /s) | | |

- 8-bit quantization about doubles the speed. The 8-bit models here were calibrated on random images: the timing is
  valid, the detections are not. A real model needs calibrating on real images and its accuracy checking.
- Peak memory of the benchmark process: 154 MB. The `peak_rss_mb` of each row in `onnx.json` is `ru_maxrss`, the
  process's running peak, so every row after the first is the peak so far, not that model's own.
- ncnn on the CPU runs the float models faster than ONNX Runtime does (next table): 43 ms against 87 ms at 224 px.

## Vision: CPU against GPU (ncnn, float models)

| Model | Input | CPU, 4 cores | GPU (Adreno 702, Vulkan) | GPU slower by |
|---|---|---|---|---|
| Classification | 224 px | 25 ms (39.9 /s) | 183 ms (5.5 /s) | 7.3× |
| Detection | 160 px | 28 ms (36.0 /s) | 190 ms (5.3 /s) | 6.8× |
| Detection | 224 px | 43 ms (23.3 /s) | 312 ms (3.2 /s) | 7.3× |
| Detection | 320 px | 81 ms (12.4 /s) | 591 ms (1.7 /s) | 7.3× |
| Detection | 640 px | 324 ms (3.1 /s) | 2,174 ms (0.5 /s) | 6.7× |
| Pose | 320 px | 87 ms (11.5 /s) | 794 ms (1.3 /s) | 9.1× |
| Segmentation | 320 px | 107 ms (9.3 /s) | 720 ms (1.4 /s) | 6.7× |

- Turning half-precision off on the GPU made no real difference (293 ms against 301 ms at 224 px).
- The first GPU run of a model takes up to 12 s while shaders compile.
- The one thing the GPU offers: it leaves the four CPU cores free and runs cooler (about 52 °C against 65–69 °C).
- CPU timings moved between runs: the same 224 px model measured 43 ms in one run and 52–92 ms in a later one, after
  the GPU tests. The cause was not found (heat or frequency scaling are the suspects). Re-measure before relying on a
  frame rate.

## Language models (llama.cpp, CPU, 4 threads, 4-bit weights)

Prompt: a child greets the robot; the model must reply in two short sentences and name one action from a list of
five. 64 tokens generated.

| Model | Reading the prompt | Writing | Reply to the test prompt |
|---|---|---|---|
| SmolLM2 135M | 52.0 tokens/s | 28.0 tokens/s | Fluent, but named no action |
| SmolLM2 360M | 18.4 | 12.4 | Repeated the child's words; no action |
| Qwen2.5 0.5B | 16.6 | 9.8 | "Quack, quack, quack!"; no action named |
| Qwen2.5 1.5B | 5.4 | 3.5 | Said it cannot do any of the actions |
| Qwen2.5 3B | 2.5 | 1.8 | "Duck, I am here! Quack." and chose `walk_forward`: the only one to follow the instruction |

One prompt is not an evaluation, but the pattern is the expected one: the models fast enough for conversation
(10–28 tokens a second) did not follow the instruction, and the one that did needs about 25 s to read a 60-token prompt
and 10 s per short sentence. Memory was never the limit (the 3B model left 3.1 GB available).

**On the GPU (llama.cpp's Vulkan build): fails.** Every model, from 135M to 3B, aborted with
`vk::DeviceLostError`, and the kernel log shows the GPU driver hanging and recovering (`a6xx_recover`). The board
stayed up and the robot's firmware kept answering.

## Is there an accelerator we are not using?

Checked on the board and against Arduino's, Qualcomm's and other people's material (2026-11-14). No.

- **No NPU.** An Arduino forum reply puts it plainly: "the QRB2210 doesn't have any TPU (so there are no TOPS
  specs)". On the board, the only co-processors Linux starts are the modem and the audio DSP (`adsp`); there is no
  compute DSP and no `fastrpc` device, which is what Qualcomm's AI runtime (QNN) needs.
- **Arduino App Lab runs its AI blocks on the CPU on this board.** Each block ships two container recipes: one for
  the UNO Q (`ei-models-runner`, no device access at all) and one for a different board, the VENTUNO Q
  (`ei-qnn-models-runner`, which maps the compute DSP). In App Lab's model list every QNN-accelerated model, every
  local language and vision-language model (Qwen, run by Qualcomm's Genie), speech recognition and text-to-speech is
  marked `supported_boards: ["ventunoq"]`. The VENTUNO Q is Arduino's larger board (Dragonwing IQ8, an NPU rated at
  40 TOPS, 16 GB of memory).
- **The GPU has only the open Mesa drivers** (Turnip for Vulkan, rusticl for OpenCL). Qualcomm's own GPU drivers
  and AI runtime are not on the board's Debian image.
- **Other people measure the same thing.** An independent project found ncnn on the GPU 5.7 times slower than the
  CPU for a face detector (395 ms against 69 ms), and the same `ErrorDeviceLost` crash with llama.cpp on Vulkan; a
  second project using MNN over OpenCL found the time dominated by copying between GPU and CPU.
- **One path is untested:** a reviewer reports that Edge Impulse can export a model for "Arduino UNO Q (GPU)". Edge
  Impulse's own UNO Q page does not document it and nobody has published a speed for it. It needs an Edge Impulse
  project to try.

What the GPU is good for on this board: drawing (a display, a face, a web UI) and keeping the CPU cores free and
cooler. The board also has a hardware video encoder.

Sources: [Arduino forum, "How many TOPs?"](https://forum.arduino.cc/t/how-many-tops/1410482) ·
[Edge Impulse, UNO Q board page](https://docs.edgeimpulse.com/hardware/boards/arduino-uno-q) ·
[raspberry.tips, UNO Q as an AI camera](https://raspberry.tips/en/arduino/arduino-uno-q-smartphone-camera) ·
[ferrglez/solipsis-uno-q-edge-ai](https://github.com/ferrglez/solipsis-uno-q-edge-ai) ·
[vamp-carti/Textile-defect-detection](https://github.com/vamp-carti/Textile-defect-detection) ·
[owhinata/zaitaku-mealpace issue 51](https://github.com/owhinata/zaitaku-mealpace/issues/51) ·
[CNX Software on the VENTUNO Q](https://cnx-software.com/2026/08/25/299-arduino-ventuno-q-sbc-combines-qualcomm-dragonwing-iq8-soc-and-stm32h5-mcu/)

## Heat

With all four cores busy the hottest sensor rose from 44 °C to 68 °C in about five minutes, bare board in open air.
Where it starts to slow itself down was not found. Inside a closed robot body next to a battery it will run hotter;
measure there before choosing a frame rate.

## What this means for the robot

- **Do:** run the walking policy on the Linux side at 50–100 Hz on one core; run one small 8-bit vision model
  (detection at 160–224 px, or classification) on two cores at 15–27 frames a second; keep one core for the system.
- **Possible, slowly:** pose or segmentation at 320 px at 4–5 frames a second; a 3B language model for a spoken reply
  every half minute or so, not for control.
- **Do not plan on:** the GPU for neural networks with today's open drivers; a language model in the control loop.
- **For language and reasoning,** use the network: the board has Wi-Fi, and Arduino's "Cloud LLM" building block exists
  for this. A small on-board model can still pick between a few fixed phrases.

## Not measured

- A real camera pipeline (capture, resize, decode): no camera was attached. Expect it to cost part of a core.
- Vision and policy running at the same time, and anything over more than a few minutes.
- Vision-language models (image + text), speech recognition and speech output.
- OpenCL on the GPU, and Qualcomm's own drivers and AI runtime, which may be much faster than Mesa's but are not
  installed on the board.
- The microcontroller side (STM32): whether a small policy could run there.
- Arduino App Lab's ready-made AI building blocks (object detection, keyword spotting and others).
- Accuracy of anything.

## Reproducing

The scripts are in [`scripts/uno-q-bench/`](../../scripts/uno-q-bench/). On a PC, build the test models (the YOLO weights are AGPL-3.0, fetched by the `ultralytics` package, and are not
stored in this repository):

```sh
uv run --with onnx --with numpy --with onnxruntime --with ultralytics --with onnxslim python make_models.py out/
```

For the GPU table, export the same weights with Ultralytics' ncnn export into `ncnn_models/<name>_<size>/`.

On the board (reached with `adb`, see [build-and-flash.md](../robot/build-and-flash.md)), in a work folder such as `~/bench`:

```sh
# CPU, ONNX Runtime: inside Arduino's app container, which has a Python package installer
docker run --rm --entrypoint sh -v ~/bench:/bench -v /sys/class/thermal:/sys/class/thermal:ro \
  ghcr.io/arduino/app-bricks/python-apps-base:0.13.2 -c \
  "cd /bench && uv venv -q venv && uv pip install -q --python venv/bin/python onnxruntime numpy && \
   venv/bin/python bench_onnx.py models --json results/onnx.json"

# CPU against GPU, ncnn: on the host, because the container cannot see the GPU drivers.
# The board has no pip; copy `uv` out of the container first, then:
./uv venv -q --python /usr/bin/python3 hostvenv && ./uv pip install -q --python hostvenv/bin/python ncnn numpy
hostvenv/bin/python bench_ncnn.py ncnn_models --json results/ncnn.json

# Language models (downloads llama.cpp and about 4 GB of models; delete llm/models afterwards)
sh bench_llm.sh ~/bench/llm cpu
sh bench_llm.sh ~/bench/llm vulkan      # crashes the GPU driver on this board; it recovers
```

Raw numbers: [`onnx.json`](../../scripts/uno-q-bench/results/onnx.json), [`ncnn.json`](../../scripts/uno-q-bench/results/ncnn.json).
