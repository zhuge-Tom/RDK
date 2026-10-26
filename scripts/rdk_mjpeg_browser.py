#!/usr/bin/env python3
"""Serve an annotated JPEG stream produced by the direct MVS C++ detector."""

from __future__ import annotations

from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import select
import sys
import threading
import time


PAGE = """<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>MVS C++ 蓝色装甲检测</title><style>
*{box-sizing:border-box}body{margin:0;background:#0c0f11;color:white;font:16px system-ui,sans-serif}
header{min-height:64px;display:flex;align-items:center;gap:22px;padding:10px 20px;background:#20252a;border-bottom:1px solid #4b535b;flex-wrap:wrap}
strong{font-size:20px}#state{padding:8px 18px;border-radius:8px;font-weight:700}.found{background:#087f23}.lost{background:#b42318}
#details,#corners{color:#d3d9de}#corners{font:13px ui-monospace,monospace}main{height:calc(100vh - 84px);display:grid;place-items:center}
img{display:block;max-width:100%;max-height:100%;object-fit:contain}</style>
<header><strong>MVS · C++ 蓝色装甲检测</strong><span id="state" class="lost">等待画面</span><span id="details"></span><span id="corners"></span></header>
<main><img src="/stream" alt="实时检测画面"></main><script>
async function update(){try{const v=await(await fetch('/status',{cache:'no-store'})).json();const s=document.getElementById('state');s.className=v.detected?'found':'lost';s.textContent=v.detected?`已识别：${v.armors} 个蓝色装甲板`:'未识别到装甲板';document.getElementById('details').textContent=`${v.corners.length} 个角点 · 检测 ${v.detect_fps.toFixed(1)} FPS · 显示 ${v.web_fps.toFixed(1)} FPS`;document.getElementById('corners').textContent=v.corners.map((p,i)=>`P${i+1}(${p[0].toFixed(0)},${p[1].toFixed(0)})`).join(' ')}catch(_){const s=document.getElementById('state');s.className='lost';s.textContent='连接中断'}}update();setInterval(update,250);
</script></html>"""


class State:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.jpeg: bytes | None = None
        self.sequence = 0
        self.times: deque[float] = deque(maxlen=30)

    def web_fps(self) -> float:
        if len(self.times) < 2:
            return 0.0
        return (len(self.times) - 1) / (self.times[-1] - self.times[0])


state = State()
status_path = Path("/tmp/rmcs-blue-armor-status.json")
stopping = threading.Event()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, _format: str, *_args: object) -> None:
        pass

    def send_bytes(self, payload: bytes, content_type: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self) -> None:
        if self.path == "/":
            self.send_bytes(PAGE.encode(), "text/html; charset=utf-8")
            return
        if self.path == "/status":
            try:
                value = json.loads(status_path.read_text())
            except (OSError, json.JSONDecodeError):
                value = {"detected": False, "armors": 0, "corners": [], "detect_fps": 0.0}
            value["web_fps"] = state.web_fps()
            self.send_bytes(json.dumps(value).encode(), "application/json")
            return
        if self.path != "/stream":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        seen = 0
        try:
            while True:
                with state.condition:
                    state.condition.wait_for(lambda: state.sequence > seen, timeout=5.0)
                    jpeg = state.jpeg
                    seen = state.sequence
                if jpeg is None:
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass


def read_stream() -> None:
    buffer = bytearray()
    descriptor = sys.stdin.buffer.fileno()
    while not stopping.is_set():
        readable, _, _ = select.select([descriptor], [], [], 0.5)
        if not readable:
            continue
        chunk = os.read(descriptor, 65536)
        if not chunk:
            return
        buffer.extend(chunk)
        while True:
            begin = buffer.find(b"\xff\xd8")
            if begin < 0:
                buffer[:] = buffer[-1:]
                break
            end = buffer.find(b"\xff\xd9", begin + 2)
            if end < 0:
                del buffer[:begin]
                break
            jpeg = bytes(buffer[begin : end + 2])
            del buffer[: end + 2]
            with state.condition:
                state.jpeg = jpeg
                state.sequence += 1
                state.times.append(time.monotonic())
                state.condition.notify_all()


def main() -> int:
    reader = threading.Thread(target=read_stream)
    reader.start()
    bind = os.environ.get("BROWSER_BIND", "127.0.0.1")
    port = int(os.environ.get("BROWSER_PORT", "8876"))
    server = ThreadingHTTPServer((bind, port), Handler)
    server.daemon_threads = True
    print(f"Browser: http://{bind}:{port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stopping.set()
        server.server_close()
        reader.join(timeout=1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
