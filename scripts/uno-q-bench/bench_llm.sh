#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# Time small language models on the board with llama.cpp's prebuilt ARM Linux binaries.
# Run ON the board:   sh bench_llm.sh <work dir> [cpu|vulkan]
# Downloads llama.cpp (about 20 MB) and each model from Hugging Face into <work dir>, then prints llama.cpp's own
# reply and its timing line (prompt tokens per second, generated tokens per second) for each model.
# Needs internet and about 4 GB free for the full list; delete <work dir>/models afterwards.
set -u
W=${1:-$HOME/bench/llm}; KIND=${2:-cpu}
mkdir -p "$W" && cd "$W" || exit 1
# The newest build tag (bNNNN). "releases/latest" is not used: it can point at a non-build release.
TAG=${LLAMA_TAG:-}
if [ -z "$TAG" ]; then
  TAG=$(curl -s "https://api.github.com/repos/ggml-org/llama.cpp/releases?per_page=10" | grep -o '"tag_name": "b[0-9]*"' | head -1 | cut -d'"' -f4)
fi
[ -n "$TAG" ] || { echo "could not find a llama.cpp build tag"; exit 1; }
case "$KIND" in vulkan) PKG="llama-$TAG-bin-ubuntu-vulkan-arm64.tar.gz";; *) PKG="llama-$TAG-bin-ubuntu-arm64.tar.gz";; esac
if [ -z "$(find "$W/llama-$KIND" -name llama-cli -type f 2>/dev/null | head -1)" ]; then
  curl -sL -o pkg.tgz "https://github.com/ggml-org/llama.cpp/releases/download/$TAG/$PKG" || exit 1
  mkdir -p "$W/llama-$KIND" && tar xzf pkg.tgz -C "$W/llama-$KIND" && rm -f pkg.tgz
fi
B=$(find "$W/llama-$KIND" -name llama-cli -type f | head -1)
[ -n "$B" ] || { echo "llama-cli not found after unpacking $PKG"; exit 1; }
D=$(dirname "$B")
export LD_LIBRARY_PATH="$D" LLAMA_CACHE="$W/models"
echo "llama.cpp $TAG ($KIND) in $D"
NGL=0; [ "$KIND" = vulkan ] && NGL=99
PROMPT="A child says to a small walking duck robot: 'Hello duck, can you come here and say hi?' Write the duck's reply in two short sentences, then name one action from this list: walk_forward, turn_left, turn_right, sit, quack."
run() {  # run <label> <hf repo:quant> <threads>
  echo "=== $1 | $2 | threads $3 | $KIND"
  "$B" -hf "$2" -t "$3" -ngl $NGL -c 1024 -n 64 --temp 0 -st --no-display-prompt --simple-io -p "$PROMPT" > run.log 2>&1 < /dev/null
  echo "    exit code $?"
  # llama-cli prints "[ Prompt: N t/s | Generation: N t/s ]" after the reply; the reply is the text between the echoed prompt and that line.
  awk '/^> /{on=1; next} /^\[ Prompt:/{on=0} on && NF' run.log | head -6 | cut -c1-160 | sed 's/^/    reply: /'
  grep -E "^\[ Prompt:|rror|illegal|Killed|failed" run.log | cut -c1-150 | sed 's/^/    /' 
  free -m | awk 'NR==2{print "    board memory used " $3 " MB, available " $7 " MB"}'
  cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -n | tail -1 | awk '{print "    hottest sensor " $1/1000 " C"}'
}
T=${3:-4}
run "SmolLM2 135M" "bartowski/SmolLM2-135M-Instruct-GGUF:Q4_K_M" "$T"
run "SmolLM2 360M" "bartowski/SmolLM2-360M-Instruct-GGUF:Q4_K_M" "$T"
run "Qwen2.5 0.5B" "Qwen/Qwen2.5-0.5B-Instruct-GGUF:Q4_K_M" "$T"
run "Qwen2.5 1.5B" "Qwen/Qwen2.5-1.5B-Instruct-GGUF:Q4_K_M" "$T"
run "Qwen2.5 3B"   "Qwen/Qwen2.5-3B-Instruct-GGUF:Q4_K_M" "$T"
du -sh "$W/models" 2>/dev/null
