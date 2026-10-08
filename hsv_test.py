from flask import Flask, Response, render_template_string, request
import cv2
import numpy as np
from picamera2 import Picamera2

app = Flask(__name__)

# 1. 初始化樹莓派相機 (Picamera2)
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (320, 240), "format": "RGB888"}))
picam2.start()

# 預設橘色球體的 HSV 初始範圍
hsv_params = {
    'h_min': 5,  'h_max': 25,
    's_min': 100, 's_max': 255,
    'v_min': 100, 'v_max': 255
}

# 網頁介面與即時滑動條前端
HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>PiCamera2 HSV Tuner</title>
    <style>
        body { font-family: sans-serif; text-align: center; background: #1a1a1a; color: #fff; margin: 0; padding: 20px; }
        .container { max-width: 700px; margin: auto; }
        img { width: 100%; max-width: 640px; border: 3px solid #444; border-radius: 8px; }
        .controls { background: #2a2a2a; padding: 15px; border-radius: 8px; margin-top: 15px; text-align: left; display: inline-block; width: 100%; box-sizing: border-box;}
        .slider-group { margin: 10px 0; display: flex; align-items: center; justify-content: space-between; }
        label { width: 60px; font-weight: bold; }
        input[type=range] { flex-grow: 1; margin: 0 10px; }
        span { width: 40px; text-align: right; }
    </style>
</head>
<body>
    <div class="container">
        <h2>樹莓派橘球 HSV 調整工具</h2>
        <p>左側：原色畫面 | 右側：遮罩 Mask (請調整數值讓橘球變白、背景變黑)</p>
        <div>
            <img src="/video_feed">
        </div>
        <div class="controls">
            <div class="slider-group"><label>H Min</label><input type="range" id="h_min" min="0" max="179" value="5" oninput="updateValues()"><span id="v_h_min">5</span></div>
            <div class="slider-group"><label>H Max</label><input type="range" id="h_max" min="0" max="179" value="25" oninput="updateValues()"><span id="v_h_max">25</span></div>
            <div class="slider-group"><label>S Min</label><input type="range" id="s_min" min="0" max="255" value="100" oninput="updateValues()"><span id="v_s_min">100</span></div>
            <div class="slider-group"><label>S Max</label><input type="range" id="s_max" min="0" max="255" value="255" oninput="updateValues()"><span id="v_s_max">255</span></div>
            <div class="slider-group"><label>V Min</label><input type="range" id="v_min" min="0" max="255" value="100" oninput="updateValues()"><span id="v_v_min">100</span></div>
            <div class="slider-group"><label>V Max</label><input type="range" id="v_max" min="0" max="255" value="255" oninput="updateValues()"><span id="v_v_max">255</span></div>
        </div>
    </div>

    <script>
        function updateValues() {
            let params = {};
            ['h_min', 'h_max', 's_min', 's_max', 'v_min', 'v_max'].forEach(id => {
                let val = document.getElementById(id).value;
                document.getElementById('v_' + id).innerText = val;
                params[id] = val;
            });

            // 改用 POST JSON 傳遞，避免亂碼與 405 問題
            fetch('/update', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(params)
            });
        }
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)


@app.route('/update', methods=['POST'])
def update():
    data = request.get_json()
    if data:
        for key in hsv_params:
            if key in data:
                hsv_params[key] = int(data[key])
    return "OK"

def generate_frames():
    while True:
        # 從 Picamera2 取得畫面 (RGB) 並轉為 OpenCV 的 BGR
        frame = picam2.capture_array()
        # 轉換成 HSV 空間
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 建立遮罩
        lower = np.array([hsv_params['h_min'], hsv_params['s_min'], hsv_params['v_min']])
        upper = np.array([hsv_params['h_max'], hsv_params['s_max'], hsv_params['v_max']])
        mask = cv2.inRange(hsv, lower, upper)
        
        # 將黑白遮罩轉成 3 通道，才能跟彩色原圖並排拼接
        mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        #combined = np.hstack((frame, mask_3ch))
        
        # 壓縮成 JPEG 串流格式傳送至網頁
        ret, buffer = cv2.imencode('.jpg', mask_3ch)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # 啟動網頁伺服器，監聽所有外部連線
    app.run(host='0.0.0.0', port=5000, debug=False)