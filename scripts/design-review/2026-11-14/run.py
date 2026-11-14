# SPDX-License-Identifier: Apache-2.0
# Headless runner: blender -b file.blend --python run.py -- tool.py [args.json]
import sys, json, bpy, traceback
argv = sys.argv[sys.argv.index("--") + 1:]
tool = argv[0]
G = {"__name__": "__main__", "bpy": bpy}
G["ARGS"] = json.load(open(argv[1])) if len(argv) > 1 else {}
try:
    exec(compile(open(tool).read(), tool, "exec"), G)
    for k in ("result", "RESULT", "out", "OUT"):
        if k in G and not callable(G[k]):
            print(f"##{k}##", json.dumps(G[k], default=str)[:200000])
except Exception:
    traceback.print_exc()
    sys.exit(3)
