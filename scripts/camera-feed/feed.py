#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Serve a stream of JPEG pictures arriving on standard input as a live video on this computer only.

The robot's camera is read on the board and its pictures come here through SSH, so nothing on the board listens
on the network. What this serves (http://127.0.0.1:PORT/stream, multipart JPEG) is a URL trainnr's camera capture
and any browser can show:

    ssh USER@BOARD "v4l2-ctl -d /dev/video2 --set-fmt-video=width=640,height=480,pixelformat=MJPG \
        --set-parm=15 --stream-mmap --stream-to=-" | python3 scripts/camera-feed/feed.py --port 8091

    start_capture(name="...", source="camera", url="http://127.0.0.1:8091/stream")     (trainnr)

Which /dev/videoN is the camera can change from one boot to the next: `duckd see` prints the one it found.
Needs only Python's standard library. It ends when the pictures stop.
"""
import argparse
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SOI, EOI = b"\xff\xd8", b"\xff\xd9"
latest = None  # the newest whole picture
arrived = threading.Condition()
ended = False


def read_pictures(source):
    """Cut the byte stream into pictures: each starts with the JPEG start marker and ends with the end marker."""
    global latest, ended
    held = b""
    while chunk := source.read(65536):
        held += chunk
        while True:
            start = held.find(SOI)
            end = held.find(EOI, start + 2) if start >= 0 else -1
            if end < 0:
                break
            with arrived:
                latest = held[start:end + 2]
                arrived.notify_all()
            held = held[end + 2:]
    with arrived:
        ended = True
        arrived.notify_all()


class Feed(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.split("?")[0] != "/stream":
            self.send_error(404, "the video is at /stream")
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=picture")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        sent = None
        try:
            while True:
                with arrived:
                    arrived.wait_for(lambda: ended or latest is not sent)
                    if ended:
                        return
                    sent = latest
                self.wfile.write(b"--picture\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(sent))
                self.wfile.write(sent + b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass  # the viewer went away

    def log_message(self, *_):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=8091)
    args = parser.parse_args()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Feed)  # this computer only
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"live at http://127.0.0.1:{args.port}/stream", file=sys.stderr, flush=True)
    read_pictures(sys.stdin.buffer)
    print("the pictures stopped", file=sys.stderr)


if __name__ == "__main__":
    main()
