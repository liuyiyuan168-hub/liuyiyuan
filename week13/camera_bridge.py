#!/usr/bin/env python3
import os
import threading
from datetime import datetime
from pathlib import Path

# WSLg 下让 OpenCV/Qt 直接走 xcb。
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
os.environ.setdefault("QT_X11_NO_MITSHM", "1")

import cv2
import numpy as np
from flask import Flask, Response, render_template_string
from flask_socketio import SocketIO


HOST = "0.0.0.0"
PORT = 5000

# 这里的具体字典必须与生成 marker 时使用的具体字典一致。
ARUCO_DICT = cv2.aruco.DICT_4X4_50
EXPECTED_MARKER_ID = 6
FRAME_JPEG_QUALITY = 0.8

SCRIPT_DIR = Path(__file__).resolve().parent


def prepare_save_dir():
    candidates = [
        Path.cwd() / "calib_images",
        Path.home() / "ai-robotics-data" / "week12" / "calib_images",
    ]

    for candidate in candidates:
        try:
            candidate.mkdir(parents=True, exist_ok=True)
            probe = candidate / ".write_test"
            probe.write_text("ok", encoding="utf-8")
            probe.unlink()
            return candidate
        except OSError:
            continue

    raise RuntimeError(
        "Cannot create a writable calib_images directory. "
        "Please check directory permissions."
    )


SAVE_DIR = prepare_save_dir()

app = Flask(__name__)
socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    max_http_buffer_size=8_000_000,
    async_mode="threading",
)

aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

frame_lock = threading.Lock()
latest_frame = None
latest_display = None
latest_status = {
    "detected": 0,
    "rejected": 0,
    "expected_id": EXPECTED_MARKER_ID,
    "matched_expected": False,
    "save_dir": str(SAVE_DIR),
}


HTML = f"""
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Week 12 Camera Bridge</title>
  <style>
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      min-height: 100vh;
      background: #0f172a;
      color: #e2e8f0;
      font-family: Arial, "Microsoft YaHei", sans-serif;
      display: flex;
      justify-content: center;
      align-items: center;
      padding: 20px;
    }}
    .panel {{
      width: min(720px, 100%);
      background: #1e293b;
      border: 1px solid #334155;
      border-radius: 12px;
      padding: 20px;
    }}
    h1 {{
      margin: 0 0 10px;
      font-size: 22px;
    }}
    p {{
      line-height: 1.6;
      color: #cbd5e1;
    }}
    .hint {{
      background: #020617;
      border-radius: 8px;
      padding: 12px;
      margin: 14px 0;
      font-size: 14px;
      line-height: 1.6;
    }}
    .status {{
      margin: 14px 0;
      padding: 10px 12px;
      border-radius: 8px;
      background: #020617;
      font-size: 14px;
    }}
    .connected {{ color: #86efac; }}
    .disconnected {{ color: #fca5a5; }}
    .preview-grid {{
      display: grid;
      grid-template-columns: 1fr;
      gap: 16px;
    }}
    .preview-card {{
      background: #0f172a;
      border: 1px solid #334155;
      border-radius: 10px;
      padding: 12px;
    }}
    .preview-card h2 {{
      margin: 0 0 10px;
      font-size: 16px;
    }}
    video, img {{
      border-radius: 10px;
      background: #000;
      display: block;
      width: 100%;
      min-height: 220px;
    }}
    .button-row {{
      display: flex;
      gap: 12px;
      margin-top: 12px;
    }}
    button {{
      margin-top: 12px;
      min-height: 44px;
      border: none;
      border-radius: 8px;
      padding: 0 14px;
      font-size: 16px;
      color: white;
      background: #2563eb;
      flex: 1;
    }}
    .metrics {{
      margin-top: 12px;
      padding: 12px;
      border-radius: 8px;
      background: #020617;
      font-size: 14px;
      line-height: 1.7;
      white-space: pre-wrap;
    }}
    @media (min-width: 900px) {{
      .panel {{
        width: min(1100px, 100%);
      }}
      .preview-grid {{
        grid-template-columns: 1fr 1fr;
      }}
    }}
  </style>
</head>
<body>
  <div class="panel">
    <h1>Week 12 手机相机桥接</h1>
    <p>允许浏览器使用后置摄像头，并把画面持续发送到 WSL 服务端。左侧是手机本地预览，右侧是服务器处理后的 ArUco 检测结果。浏览器页面保持前台，终端程序保持运行。</p>
    <div class="hint">
      课堂统一 marker 参数：<br>
      Dictionary 对应程序中的具体字典，当前脚本使用 <code>DICT_4X4_50</code><br>
      Marker ID：<code>{EXPECTED_MARKER_ID}</code>
    </div>
    <div id="status" class="status disconnected">等待连接</div>
    <div class="preview-grid">
      <div class="preview-card">
        <h2>手机本地视频</h2>
        <video id="video" autoplay playsinline muted></video>
      </div>
      <div class="preview-card">
        <h2>服务器检测结果</h2>
        <img id="serverPreview" alt="Server preview" src="/preview.jpg" />
      </div>
    </div>
    <canvas id="canvas" style="display:none;"></canvas>
    <div class="button-row">
      <button id="startBtn">重新请求摄像头</button>
      <button id="saveBtn">保存当前帧</button>
    </div>
    <div id="metrics" class="metrics">等待服务器状态...</div>
  </div>

  <script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>
  <script>
    const statusEl = document.getElementById("status");
    const metricsEl = document.getElementById("metrics");
    const video = document.getElementById("video");
    const canvas = document.getElementById("canvas");
    const serverPreview = document.getElementById("serverPreview");
    const ctx = canvas.getContext("2d");
    const socket = io({{ transports: ["polling"], upgrade: false }});
    let sending = false;

    function setStatus(text, connected) {{
      statusEl.textContent = text;
      statusEl.className = connected ? "status connected" : "status disconnected";
    }}

    async function startCamera() {{
      try {{
        const stream = await navigator.mediaDevices.getUserMedia({{
          video: {{
            width: 1280,
            height: 720,
            facingMode: {{ ideal: "environment" }}
          }},
          audio: false
        }});

        video.srcObject = stream;
        await video.play();
        setStatus("摄像头已打开，等待视频发送", true);

        if (!sending) {{
          sending = true;
          sendFrames();
        }}
      }} catch (error) {{
        setStatus("摄像头权限失败，请检查浏览器权限", false);
        console.error(error);
      }}
    }}

    function sendFrames() {{
      setInterval(() => {{
        if (!video.videoWidth || socket.disconnected) {{
          return;
        }}

        canvas.width = video.videoWidth;
        canvas.height = video.videoHeight;
        ctx.drawImage(video, 0, 0);

        canvas.toBlob(async (blob) => {{
          if (!blob || socket.disconnected) {{
            return;
          }}
          const arrayBuffer = await blob.arrayBuffer();
          socket.emit("video_frame", arrayBuffer);
        }}, "image/jpeg", {FRAME_JPEG_QUALITY});
      }}, 100);
    }}

    socket.on("connect", () => {{
      setStatus("已连接到 WSL 服务，准备打开摄像头", true);
    }});

    socket.on("disconnect", () => {{
      setStatus("与 WSL 服务断开连接", false);
    }});

    socket.on("server_status", (payload) => {{
      metricsEl.textContent =
        `Detected: ${{payload.detected}}\\n` +
        `Rejected: ${{payload.rejected}}\\n` +
        `Expected ID: ${{payload.expected_id}}\\n` +
        `Matched expected: ${{payload.matched_expected}}\\n` +
        `Save dir: ${{payload.save_dir}}`;
      serverPreview.src = `/preview.jpg?ts=${{Date.now()}}`;
    }});

    document.getElementById("startBtn").addEventListener("click", startCamera);
    document.getElementById("saveBtn").addEventListener("click", () => {{
      if (socket.connected) {{
        socket.emit("save_frame");
      }}
    }});

    setInterval(() => {{
      serverPreview.src = `/preview.jpg?ts=${{Date.now()}}`;
    }}, 500);

    startCamera();
  </script>
</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(HTML)


@socketio.on("connect")
def on_connect():
    print("client connected")


@socketio.on("disconnect")
def on_disconnect():
    print("client disconnected")


@socketio.on("video_frame")
def handle_frame(image_bytes):
    global latest_frame, latest_display, latest_status
    nparr = np.frombuffer(image_bytes, np.uint8)
    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if frame is None:
        return

    display, status = annotate_frame(frame)

    with frame_lock:
        latest_frame = frame
        latest_display = display
        latest_status = status

    socketio.emit("server_status", status)


@socketio.on("save_frame")
def on_save_frame():
    filename = save_current_frame()
    if filename is not None:
        print(f"saved: {filename}")
        socketio.emit("server_status", latest_status)


def annotate_frame(frame):
    display = frame.copy()
    gray = cv2.cvtColor(display, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)
    detected_ids = []

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(display, corners, ids)

        for i, marker_id in enumerate(ids.flatten()):
            marker_id = int(marker_id)
            detected_ids.append(marker_id)
            pts = corners[i][0]
            cx = int(pts[:, 0].mean())
            cy = int(pts[:, 1].mean())
            color = (0, 255, 0) if marker_id == EXPECTED_MARKER_ID else (0, 200, 255)
            cv2.putText(
                display,
                f"ID: {marker_id}",
                (cx, cy),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
            )

    cv2.putText(
        display,
        f"Dictionary: DICT_4X4_50  Expected ID: {EXPECTED_MARKER_ID}",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        display,
        f"Detected: {0 if ids is None else len(ids)}  Rejected: {len(rejected)}",
        (20, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        display,
        "Press s to save current frame. Stop with Ctrl+C in terminal.",
        (20, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )
    status = {
        "detected": len(detected_ids),
        "rejected": len(rejected),
        "expected_id": EXPECTED_MARKER_ID,
        "matched_expected": EXPECTED_MARKER_ID in detected_ids,
        "save_dir": str(SAVE_DIR),
    }
    return display, status


def make_waiting_preview():
    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(
        blank,
        "Waiting for phone camera frames...",
        (50, 220),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        blank,
        f"Open https://<TAILSCALE_IP>:{PORT} on the phone browser",
        (30, 280),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )
    return blank


def jpeg_response(image):
    ok, encoded = cv2.imencode(".jpg", image)
    if not ok:
        return Response(status=500)
    return Response(encoded.tobytes(), mimetype="image/jpeg")


def save_current_frame():
    with frame_lock:
        frame_to_save = None if latest_display is None else latest_display.copy()

    if frame_to_save is None:
        return None

    filename = SAVE_DIR / f"frame_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
    cv2.imwrite(str(filename), frame_to_save)
    return filename


@app.route("/preview.jpg")
def preview():
    with frame_lock:
        frame = None if latest_display is None else latest_display.copy()

    if frame is None:
        frame = make_waiting_preview()
    return jpeg_response(frame)


def main():
    print(f"Camera bridge listening on https://{HOST}:{PORT}")
    print("Open the page in the phone browser and keep this terminal running.")
    print(f"Saved frames will be written to: {SAVE_DIR}")
    socketio.run(app, host=HOST, port=PORT, ssl_context="adhoc")


if __name__ == "__main__":
    main()
