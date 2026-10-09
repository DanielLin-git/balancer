
# web_stream.py

import time
import threading

import cv2
from flask import Flask, Response


class WEBSTREAM:
    def __init__(self, host="0.0.0.0", port=5000):
        self.host = host
        self.port = port

        self.app = Flask(__name__)

        # 儲存最新一張 JPEG，而不是累積所有影像
        self.latest_jpeg = None
        self.frame_lock = threading.Lock()

        # 註冊 Flask 路由
        self._register_routes()

    def update_frame(self, frame):
        """接收處理後的 OpenCV 影像，更新最新 JPEG。"""

        success, encoded = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, 60]
        )

        if not success:
            return False

        frame_bytes = encoded.tobytes()

        with self.frame_lock:
            self.latest_jpeg = frame_bytes

        return True

    def generate_mjpeg(self):
        """持續將最新 JPEG 傳送給瀏覽器。"""

        while True:
            with self.frame_lock:
                frame_bytes = self.latest_jpeg

            # 相機尚未產生第一張影像
            if frame_bytes is None:
                time.sleep(0.01)
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + frame_bytes
                + b"\r\n"
            )

            # 限制網頁更新頻率約 30 FPS
            time.sleep(0.03)

    def _register_routes(self):
        @self.app.route("/")
        def index():
            return """
            <!DOCTYPE html>
            <html>
                <head>
                    <title>Raspberry Pi Camera</title>
                </head>
                <body>
                    <h1>Raspberry Pi Camera Live</h1>
                    <img src="/video_feed" width="640">
                </body>
            </html>
            """

        @self.app.route("/video_feed")
        def video_feed():
            return Response(
                self.generate_mjpeg(),
                mimetype=(
                    "multipart/x-mixed-replace; boundary=frame"
                )
            )

    def run(self):
        self.app.run(
            host=self.host,
            port=self.port,
            debug=False,
            threaded=True,
            use_reloader=False
        )