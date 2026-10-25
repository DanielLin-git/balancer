
import time
import threading
import logging
import cv2
from flask import Flask, Response, jsonify, request



web_logger = logging.getLogger("werkzeug")
web_logger.setLevel(logging.WARNING)
web_logger.propagate = False

web_file_handler = logging.FileHandler(
    "web.log",
    mode="a",
    encoding="utf-8"
)

web_file_handler.setLevel(logging.INFO)
web_file_handler.setFormatter(logging.Formatter(
    "%(asctime)s - %(levelname)s : %(message)s"
))

web_logger.addHandler(web_file_handler)

class WEBSTREAM:
    def __init__(self,controller,host="0.0.0.0",port=5000,):
        self.host = host
        self.port = port
        self.controller = controller

        self.params_logger = logging.getLogger("params")

        self.app = Flask(__name__)

        self.latest_jpeg = None
        self.frame_lock = threading.Lock()

        self._register_routes()

    def update_frame(self, frame):
        success, encoded = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, 60],
        )

        if not success:
            return False

        with self.frame_lock:
            self.latest_jpeg = encoded.tobytes()

        return True

    def generate_mjpeg(self):
        while True:
            with self.frame_lock:
                frame_bytes = self.latest_jpeg

            if frame_bytes is None:
                time.sleep(0.01)
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Cache-Control: no-cache\r\n\r\n"
                + frame_bytes
                + b"\r\n"
            )

            # 僅限制網頁串流頻率，不影響控制迴圈
            time.sleep(0.03)

    def _register_routes(self):

        @self.app.route("/")
        def index():
            return """
<!DOCTYPE html>
<html lang="zh-Hant">
<head>
    <meta charset="UTF-8">
    <meta name="viewport"
          content="width=device-width, initial-scale=1">
    <title>Stewart Platform Controller</title>

    <style>
        body {
            font-family: sans-serif;
            max-width: 1000px;
            margin: 20px auto;
            padding: 0 16px;
            background: #f5f6f8;
            color: #202124;
        }

        .card {
            background: white;
            padding: 18px;
            margin-bottom: 16px;
            border-radius: 12px;
            box-shadow: 0 1px 5px #0001;
        }

        img {
            width: 100%;
            max-width: 800px;
            border-radius: 8px;
        }

        .param {
            display: grid;
            grid-template-columns: 150px 1fr 90px;
            gap: 12px;
            align-items: center;
            margin: 14px 0;
        }

        input[type=range] {
            width: 100%;
        }

        input[type=number] {
            width: 85px;
            padding: 6px;
        }

        button {
            padding: 10px 16px;
            border: 0;
            border-radius: 6px;
            cursor: pointer;
            margin-right: 8px;
        }

        .status {
            white-space: pre-wrap;
            font-family: monospace;
            line-height: 1.7;
            overflow-wrap: anywhere;
        }

        @media (max-width: 600px) {
            .param {
                grid-template-columns: 1fr 90px;
            }

            .param input[type=range] {
                grid-column: 1 / -1;
                grid-row: 2;
            }
        }
    </style>
</head>

<body>
    <h1>Stewart Platform 控制面板</h1>

    <div class="card">
        <h2>即時影像</h2>
        <img src="/video_feed" alt="Camera stream">
    </div>

    <div class="card">
        <h2>控制參數</h2>
        <p>
            拖動滑桿或輸入數值，停止調整後自動套用。
            不需要重啟 Flask。
        </p>

        <div id="params">正在讀取參數...</div>

        <button onclick="reloadParams()">重新讀取</button>
        <button onclick="resetParams()">恢復初始參數</button>

        <p id="message">等待連線...</p>
    </div>

    <div class="card">
        <h2>控制狀態</h2>
        <div id="state" class="status">等待控制資料...</div>
    </div>

<script>
const definitions = {
    kp: {
        label: "Kp",
        min: 0, max: 0.5, step: 0.001
    },
    ki: {
        label: "Ki",
        min: 0, max: 0.1, step: 0.0001
    },
    kd: {
        label: "Kd",
        min: 0, max: 0.1, step: 0.0001
    },
    tilt_gain_x: {
        label: "Tilt Gain X",
        min: 0, max: 0.2, step: 0.001
    },
    tilt_gain_y: {
        label: "Tilt Gain Y",
        min: 0, max: 0.2, step: 0.001
    },
    theta_limit: {
        label: "Theta Limit (rad)",
        min: 0.005, max: 0.15, step: 0.001
    },
    phi_limit: {
        label: "Phi Limit (rad)",
        min: 0.005, max: 0.15, step: 0.001
    }
};

let currentParams = {};
let pendingValues = {};
let debounceTimer = null;
let requestVersion = 0;

function setMessage(text, isError=false) {
    const el = document.getElementById("message");
    el.textContent = text;
    el.style.color = isError ? "#b00020" : "#176b36";
}

function renderParams(params) {
    currentParams = {...params};

    const root = document.getElementById("params");
    root.replaceChildren();

    for (const [key, def] of Object.entries(definitions)) {
        const row = document.createElement("div");
        row.className = "param";

        const label = document.createElement("label");
        label.htmlFor = "range_" + key;
        label.textContent = def.label;

        const range = document.createElement("input");
        range.type = "range";
        range.id = "range_" + key;
        range.min = def.min;
        range.max = def.max;
        range.step = def.step;
        range.value = params[key];

        const number = document.createElement("input");
        number.type = "number";
        number.id = "number_" + key;
        number.min = def.min;
        number.max = def.max;
        number.step = def.step;
        number.value = params[key];

        range.addEventListener("input", () => {
            number.value = range.value;
            queueParam(key, Number(range.value));
        });

        number.addEventListener("change", () => {
            const value = Number(number.value);

            if (!Number.isFinite(value) ||
                value < def.min || value > def.max) {
                setMessage("參數超出允許範圍：" + key, true);
                number.value = currentParams[key];
                return;
            }

            range.value = value;
            queueParam(key, value);
        });

        row.append(label, range, number);
        root.appendChild(row);
    }
}

function queueParam(key, value) {
    pendingValues[key] = value;

    setMessage("等待套用參數...");

    clearTimeout(debounceTimer);
    debounceTimer = setTimeout(sendParams, 150);
}

async function sendParams() {
    const updates = {...pendingValues};
    pendingValues = {};

    if (Object.keys(updates).length === 0) return;

    const version = ++requestVersion;

    try {
        const response = await fetch("/api/params", {
            method: "POST",
            headers: {"Content-Type": "application/json"},
            body: JSON.stringify(updates)
        });

        const data = await response.json();

        if (!response.ok) {
            throw new Error(data.error || "參數更新失敗");
        }

        currentParams = data.params;

        if (version === requestVersion) {
            setMessage("已套用至控制器：" +
                new Date().toLocaleTimeString());
        }
    } catch (error) {
        setMessage("更新失敗：" + error.message, true);
        await reloadParams();
    }

    // 如果送出期間又有新值，繼續更新
    if (Object.keys(pendingValues).length > 0) {
        clearTimeout(debounceTimer);
        debounceTimer = setTimeout(sendParams, 100);
    }
}

async function reloadParams() {
    try {
        const response = await fetch("/api/params", {
            cache: "no-store"
        });

        const data = await response.json();

        if (!response.ok) {
            throw new Error(data.error || "讀取失敗");
        }

        renderParams(data.params);
        setMessage("已讀取目前執行中的參數");
    } catch (error) {
        setMessage("讀取失敗：" + error.message, true);
    }
}

async function resetParams() {
    if (!confirm("確定恢復初始控制參數？")) return;

    const defaults = {
        "kp": 0.07,
        "ki": 0.0,
        "kd": 0.05,
        "tilt_gain_x": 0.03,
        "tilt_gain_y": 0.03,
        "theta_limit": 0.1,
        "phi_limit": 0.1,
    };

    try {
        const response = await fetch("/api/params", {
            method: "POST",
            headers: {"Content-Type": "application/json"},
            body: JSON.stringify(defaults)
        });

        const data = await response.json();

        if (!response.ok) {
            throw new Error(data.error || "重設失敗");
        }

        renderParams(data.params);
        setMessage("已恢復初始參數");
    } catch (error) {
        setMessage("重設失敗：" + error.message, true);
    }
}

async function refreshState() {
    try {
        const response = await fetch("/api/state", {
            cache: "no-store"
        });

        const data = await response.json();

        document.getElementById("state").textContent =
            JSON.stringify(data, null, 2);
    } catch (error) {
        document.getElementById("state").textContent =
            "無法取得狀態：" + error.message;
    }
}

reloadParams();
refreshState();
setInterval(refreshState, 250);
</script>
</body>
</html>
            """

        @self.app.route("/video_feed")
        def video_feed():
            response = Response(
                self.generate_mjpeg(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )
            response.headers["Cache-Control"] = (
                "no-store, no-cache, must-revalidate"
            )
            return response

        @self.app.route("/api/params", methods=["GET"])
        def get_params():
            return jsonify({
                "params": self.controller.get_params()
            })

        @self.app.route("/api/params", methods=["POST"])
        def update_params():
            data = request.get_json(silent=True)

            if not isinstance(data, dict):
                self.params_logger.warning(
                    "Invalid parameter request from %s",
                    request.remote_addr
                )

                return jsonify({
                    "error": "請提供 JSON 物件"
                }), 400

            old_params = self.controller.get_params()

            try:
                new_params = self.controller.set_params(data)

            except ValueError as exc:
                self.params_logger.warning(
                    "Parameter update rejected from %s: %s",
                    request.remote_addr,
                    str(exc)
                )

                return jsonify({
                    "error": str(exc)
                }), 400

            changed = {
                key: {
                    "old": old_params[key],
                    "new": new_params[key]
                }
                for key in data
                if old_params.get(key) != new_params.get(key)
            }

            if changed:
                self.params_logger.info(
                    "Parameter update from %s: %s",
                    request.remote_addr,
                    changed
                )

            return jsonify({
                "ok": True,
                "params": new_params
            })

        @self.app.route("/api/state", methods=["GET"])
        def get_state():
            return jsonify(self.controller.get_state())

    def run(self):
        self.app.run(
            host=self.host,
            port=self.port,
            debug=False,
            threaded=True,
            use_reloader=False,
        )
