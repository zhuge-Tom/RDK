#!/usr/bin/env python3
"""Serve an annotated JPEG stream produced by the direct MVS C++ detector."""

from __future__ import annotations

from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import select
import signal
import sys
import threading
import time


PAGE = """<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RDK 蓝色装甲定位与预测</title><style>
*{box-sizing:border-box}body{margin:0;background:#0c0f11;color:white;font:16px system-ui,sans-serif}
header{min-height:64px;display:flex;align-items:center;gap:22px;padding:10px 20px;background:#20252a;border-bottom:1px solid #4b535b;flex-wrap:wrap}
strong{font-size:20px}#state{padding:8px 18px;border-radius:8px;font-weight:700}.found{background:#087f23}.lost{background:#b42318}
#details,#corners{color:#d3d9de}#corners{font:13px ui-monospace,monospace}main{height:calc(100vh - 84px);display:grid;place-items:center}
.view-switch{display:flex;margin-left:auto;gap:4px;padding:4px;border:1px solid #445161;border-radius:9px}.view-switch button{font:inherit;font-size:13px;color:#b9c7d6;background:transparent;border:0;border-radius:6px;padding:7px 12px;cursor:pointer}.view-switch button[aria-pressed="true"]{background:#37516a;color:white}.view-switch button:focus-visible{outline:2px solid #71dbe8}
body[data-view="simple"] .debug-only{display:none!important}#legend{position:fixed;bottom:16px;left:16px;background:#131b24dc;padding:9px 13px;border-radius:8px;font-size:12px;color:#b9c7d6}body[data-view="debug"] #legend{display:none}
#geometry{position:fixed;bottom:16px;left:16px;width:min(340px,calc(100vw - 32px));max-height:45vh;overflow:auto;padding:14px 16px;background:#131b24ed;border:1px solid #344253;border-radius:12px;box-shadow:0 6px 24px #0005;font-size:13px}
.panel-head,.metric{display:flex;align-items:center;justify-content:space-between;gap:12px}.panel-head{margin-bottom:10px}.panel-head b{font-size:14px}.badge{border-radius:20px;padding:3px 9px;font-size:12px;background:#293442;color:#c7d2df}.badge.ok{background:#143d32;color:#8de5bd}.badge.warn{background:#44361e;color:#f4cd84}
.metric{padding:5px 0}.metric span{color:#9aabbc}.metric b{font:13px ui-monospace,monospace;font-variant-numeric:tabular-nums;color:#e7eef6}.metric .prediction{color:#71dbe8}
#tracking-note,#pose-note{margin:7px 0 0;color:#9aabbc;font-size:12px;line-height:1.5}#pose-note{border-top:1px solid #344253;padding-top:8px}#geometry details{margin-top:9px;color:#9aabbc}#geometry summary{cursor:pointer;font-size:12px}#geometry pre{white-space:pre-wrap;font:12px/1.7 ui-monospace,monospace;color:#c7d2df;margin:8px 0 0}
.capture{margin-top:12px;padding-top:10px;border-top:1px solid #344253}.capture button{color:#e7eef6;background:#37516a;border:0;border-radius:6px;padding:8px 10px;cursor:pointer}.capture button:disabled{opacity:.5;cursor:wait}#capture-note{display:block;color:#9aabbc;font-size:12px;line-height:1.5;margin-top:6px}
img{display:block;max-width:100%;max-height:100%;object-fit:contain}</style>
<body data-view="simple">
<header><strong>RDK 蓝色装甲 · 定位与预测</strong><span id="state" class="lost">等待画面</span>
<div class="view-switch" role="group" aria-label="显示模式"><button id="view-simple" type="button" aria-pressed="true">简洁模式</button><button id="view-debug" type="button" aria-pressed="false">调试模式</button></div>
<span id="details" class="debug-only"></span><span id="corners" class="debug-only"></span></header>
<main><img src="/stream" alt="实时检测画面"></main>
<div id="legend">绿框：装甲板 · 青色圆：预测位置</div>
<aside id="geometry" class="debug-only" aria-label="目标跟踪与预测">
<div class="panel-head"><b>目标跟踪</b><span id="track-state" class="badge">等待画面</span></div>
<div class="metric"><span>当前中心 · px</span><b id="track-center">—</b></div>
<div class="metric"><span id="prediction-label">100 ms 后 · px</span><b id="track-prediction" class="prediction">—</b></div>
<p id="tracking-note">等待目标进入画面</p>
<p id="pose-note">三维定位：等待数据</p>
<details><summary>坐标与诊断详情</summary><pre id="track-details"></pre></details>
<div class="capture"><button id="capture-button" type="button">保存标定照片</button><span id="capture-note">保存相机原始画面，不含检测标记</span></div>
</aside><script>
function setViewMode(mode) {
  const selected = mode === 'debug' ? 'debug' : 'simple';
  document.body.dataset.view = selected;
  for (const name of ['simple', 'debug']) {
    document.getElementById('view-' + name).setAttribute('aria-pressed', String(name === selected));
  }
  try { localStorage.setItem('rdk-view-mode', selected); } catch (_) {}
  if (window.location.hash !== '#' + selected) {
    window.history.replaceState(null, '', '#' + selected);
  }
}
let savedMode = 'simple';
try { savedMode = localStorage.getItem('rdk-view-mode') || 'simple'; } catch (_) {}
const linkedMode = window.location.hash.slice(1);
setViewMode(['simple', 'debug'].includes(linkedMode) ? linkedMode : savedMode);
window.addEventListener('hashchange', () => {
  const mode = window.location.hash.slice(1);
  if (['simple', 'debug'].includes(mode)) setViewMode(mode);
});
document.getElementById('view-simple').addEventListener('click', () => setViewMode('simple'));
document.getElementById('view-debug').addEventListener('click', () => setViewMode('debug'));
async function captureCalibration() {
  const button = document.getElementById('capture-button');
  const note = document.getElementById('capture-note');
  button.disabled = true;
  note.textContent = '正在保存原始画面…';
  try {
    const response = await fetch('/capture', {method: 'POST'});
    if (!response.ok) {
      const error = await response.json().catch(() => ({}));
      throw new Error(error.error || '采集请求失败，请稍后重试');
    }
    let complete = false;
    for (let i = 0; i < 20; ++i) {
      await new Promise(resolve => setTimeout(resolve, 150));
      const result = await (await fetch('/capture', {cache: 'no-store'})).json();
      if (result.pending) continue;
      if (!result.saved) throw new Error('保存失败，请检查 RDK 存储空间');
      note.textContent = `已保存 ${result.count} 张 · ${result.width} × ${result.height} · 请改变棋盘格姿态再拍`;
      complete = true;
      break;
    }
    if (!complete) throw new Error('等待相机超时，请稍后重试');
  } catch (error) { note.textContent = error.message; }
  finally { button.disabled = false; }
}
document.getElementById('capture-button').addEventListener('click', captureCalibration);
function renderGeometry(g, image) {
  const put = (id, text) => { document.getElementById(id).textContent = text; };
  const vec = (a, digits = 1) => a.map(v => v.toFixed(digits)).join(', ');
  const stale = g?.reason === 'stream_stale';
  const tracks = stale ? [] : (image?.tracks || []);
  // Prefer a confirmed, currently observed track; show every track in details.
  const t = tracks.find(t => t.observed && t.confirmed) || tracks.find(t => t.observed) || tracks[0];
  const state = document.getElementById('track-state');
  state.textContent = stale ? '连接中断' : !t ? '等待目标' : !t.observed ? '短时外推' : t.confirmed ? '稳定跟踪' : '确认目标中';
  state.className = 'badge' + (stale || (t && !t.observed) ? ' warn' : t?.confirmed ? ' ok' : '');
  const horizon = image?.prediction_horizon_ms ?? 100;
  put('prediction-label', `${horizon} ms 后 · px`);
  put('track-center', t ? vec(t.center_px) : '—');
  put('track-prediction', t?.predicted_center_px ? vec(t.predicted_center_px) : '—');
  put('tracking-note', stale ? '视频已断开，预测已清空' : !t ? '让完整的蓝色装甲进入画面' :
    `ID ${t.id} · ${tracks.length} 个目标 · ` + (!t.observed ? '目标暂时丢失，短时延续轨迹' :
    t.predicted_center_px ? '青色圆为预测中心' : '正在积累连续观测'));
  const reasons = {
    calibration_missing: '三维定位：待相机标定 · 当前显示像素坐标',
    calibration_unverified: '三维定位：标定参数待确认 · 当前显示像素坐标',
    image_aspect_mismatch: '三维定位：暂停 · 画面比例与标定不符',
    stream_stale: '三维定位：等待视频恢复'
  };
  put('pose-note', g?.reason === 'ready' ? '三维定位：已启用 · 单位 m' : reasons[g?.reason] || '三维定位：等待数据');
  const lines = ['图像坐标：原点在左上，X 向右，Y 向下'];
  for (const track of tracks) {
    lines.push(`ID ${track.id} · ${track.observed ? '当前观测' : '短时外推'}`);
    lines.push(`中心 (${vec(track.center_px)}) px`);
    lines.push(`速度 (${vec(track.velocity_px_s)}) px/s`);
    lines.push(track.predicted_center_px ? `预测 (${vec(track.predicted_center_px)}) px` : '预测：积累观测中');
  }
  if (g?.reason === 'ready') {
    lines.push('相机坐标：X 向右，Y 向下，Z 向前');
    for (const p of g.poses || []) {
      lines.push(`检测 #${p.detection_index} · XYZ (${vec(p.position_m, 3)}) m`);
      lines.push(`距离 ${p.distance_m.toFixed(3)} m · 重投影误差 ${p.reprojection_error_px.toFixed(2)} px`);
    }
    for (const track of g.tracks || []) {
      lines.push(`三维 ID ${track.id} · 速度 (${vec(track.velocity_m_s, 3)}) m/s`);
      lines.push(track.predicted_position_m ? `预测 (${vec(track.predicted_position_m, 3)}) m` : '三维预测：积累观测中');
    }
  }
  put('track-details', lines.join('\\n'));
}
async function update(){try{const v=await(await fetch('/status',{cache:'no-store'})).json();const s=document.getElementById('state');s.className=v.detected?'found':'lost';s.textContent=v.detected?`已识别：红色 ${v.red_armors || 0} 个 · 蓝色 ${v.blue_armors || 0} 个`:'未识别到装甲板';document.getElementById('details').textContent=`${v.runtime === "ros2" ? "ROS 2 · " : ""}${v.mode || "blue"} · ${v.corners.length} 个角点 · 检测 ${v.detect_fps.toFixed(1)} FPS · 显示 ${v.web_fps.toFixed(1)} FPS${v.capture_to_encode_ms != null ? ` · 采集到编码 ${v.capture_to_encode_ms.toFixed(1)} ms` : ""}`;renderGeometry(v.geometry,v.image_tracking);document.getElementById('corners').textContent=v.corners.map((p,i)=>`P${i+1}(${p[0].toFixed(0)},${p[1].toFixed(0)})`).join(' ')}catch(_){const s=document.getElementById('state');s.className='lost';s.textContent='连接中断';renderGeometry({reason:'stream_stale'})}}update();setInterval(update,250);
</script></html>"""


class State:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.jpeg: bytes | None = None
        self.sequence = 0
        self.times: deque[float] = deque(maxlen=30)

    def publish_jpeg(self, jpeg: bytes) -> None:
        with self.condition:
            self.jpeg = jpeg
            self.sequence += 1
            self.times.append(time.monotonic())
            self.condition.notify_all()

    def web_fps(self) -> float:
        if len(self.times) < 2:
            return 0.0
        return (len(self.times) - 1) / (self.times[-1] - self.times[0])


state = State()
status_path = Path(os.environ.get("ARMOR_STATUS_FILE", "/tmp/rmcs-blue-armor-status.json"))
capture_directory = os.environ.get("ARMOR_CAPTURE_DIR")
status_provider = None
capture_provider = None
stopping = threading.Event()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, _format: str, *_args: object) -> None:
        pass

    def send_bytes(self, payload: bytes, content_type: str, status: int = 200) -> None:
        self.send_response(status)
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
                value = status_provider() if status_provider else json.loads(status_path.read_text())
            except (OSError, json.JSONDecodeError):
                value = {"detected": False, "armors": 0, "corners": [], "detect_fps": 0.0}
            with state.condition:
                fresh = bool(state.times) and time.monotonic() - state.times[-1] < 2.0
                fps = state.web_fps() if fresh else 0.0
            if not fresh:
                value.update(detected=False, armors=0, corners=[], colors=[],
                             red_armors=0, blue_armors=0, detect_fps=0.0)
                value["geometry"] = {"reason": "stream_stale", "poses": [], "tracks": []}
                value["image_tracking"] = {"units": "pixels", "tracks": []}
            value["web_fps"] = fps
            value["stream_alive"] = fresh
            self.send_bytes(json.dumps(value).encode(), "application/json")
            return
        if self.path == "/capture":
            value = {"pending": False, "saved": False, "count": 0}
            if capture_directory:
                root = Path(capture_directory)
                try:
                    value.update(json.loads((root / "last_capture.json").read_text()))
                except (OSError, json.JSONDecodeError):
                    pass
                value.update(pending=(root / "capture.request").exists(),
                             count=sum(1 for _ in root.glob("frame-*.png")))
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
            while not stopping.is_set():
                with state.condition:
                    updated = state.condition.wait_for(
                        lambda: state.sequence > seen or stopping.is_set(), timeout=1.0
                    )
                    if not updated or stopping.is_set():
                        continue
                    jpeg = state.jpeg
                    seen = state.sequence
                if jpeg is None:
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def do_POST(self) -> None:
        if self.path != "/capture":
            self.send_error(404)
            return
        with state.condition:
            fresh = bool(state.times) and time.monotonic() - state.times[-1] < 2.0
        if not capture_directory or not fresh:
            message = "未配置照片保存目录" if not capture_directory else "相机画面已断开，请等待画面恢复后再拍"
            self.send_bytes(json.dumps({"error": message}).encode(), "application/json", 503)
            return
        if capture_provider:
            try:
                if not capture_provider():
                    raise RuntimeError("相机未能保存照片，请检查 RDK 终端日志")
            except Exception as error:
                self.send_bytes(json.dumps({"error": str(error)}).encode(), "application/json", 503)
                return
            self.send_bytes(b'{"pending":true}', "application/json")
            return
        try:
            root = Path(capture_directory)
            root.mkdir(parents=True, exist_ok=True)
            # One outstanding request; concurrent clicks capture the same next frame.
            if not (root / "capture.request").exists():
                (root / "last_capture.json").unlink(missing_ok=True)
            (root / "capture.request").touch(exist_ok=True)
        except OSError:
            self.send_error(503, "Capture directory is not writable")
            return
        self.send_bytes(b'{"pending":true}', "application/json")


def read_stream() -> None:
    buffer = bytearray()
    descriptor = sys.stdin.buffer.fileno()
    while not stopping.is_set():
        readable, _, _ = select.select([descriptor], [], [], 0.5)
        if not readable:
            continue
        chunk = os.read(descriptor, 65536)
        if not chunk:
            stopping.set()
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
            state.publish_jpeg(jpeg)


def main(reader_target=read_stream) -> int:
    bind = os.environ.get("BROWSER_BIND", "127.0.0.1")
    port = int(os.environ.get("BROWSER_PORT", "8876"))
    server = ThreadingHTTPServer((bind, port), Handler)
    server.daemon_threads = True
    server.timeout = 0.5
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda _signum, _frame: stopping.set())
    reader = threading.Thread(target=reader_target)
    reader.start()
    print(f"Browser: http://{bind}:{port}/", flush=True)
    try:
        while not stopping.is_set():
            server.handle_request()
    except KeyboardInterrupt:
        pass
    finally:
        stopping.set()
        server.server_close()
        reader.join(timeout=1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
