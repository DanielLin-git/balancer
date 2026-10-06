import time
import numpy as np
import cv2
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2

app = Flask(__name__)

# 初始化 Picamera2
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"}))
picam2.start()



def gen_frames():
    while True:
        # 抓取相機影像陣列
        frame = picam2.capture_array()
        if frame is None:
            break
        
        prev_time = time.time()
        prev_center = None
        center = None
        radius = None
        
        # 色彩空間轉換 (RGB 轉 BGR 給 OpenCV 辨識用)
        lower = np.array([10, 100, 100])
        upper = np.array([40, 255, 255])

        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        HSV_image = cv2.cvtColor(frame_bgr, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(HSV_image , lower, upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 3. 尋找輪廓 (比 Hough 快上好幾倍)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # 找出最大面積的輪廓（假設畫面中最大的目標就是球）
            c = max(contours, key=cv2.contourArea)
            
            # 確保面積大於一個微小閾值，避免雜訊誤判
            if cv2.contourArea(c) > 100:
                # 計算最小外接圓
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                center = (int(x), int(y))
                radius = int(radius)

                # 4. 計算速度（位置變化量 / 時間差）
                current_time = time.time()
                dt = current_time - prev_time
                if prev_center is not None and dt > 0:
                    # 像素距離換算 (pixel/sec)
                    pixel_distance = np.sqrt((center[0] - prev_center[0])**2 + (center[1] - prev_center[1])**2)
                    speed = pixel_distance / dt
                    print(f"速度: {speed:.2f} px/s, 位置: {center}")


                prev_center = center
                prev_time = current_time
            


        result = cv2.bitwise_and(frame, frame, mask=mask)
        # ==========================================
        # 在這裡加入你的 OpenCV 影像辨識與處理邏輯
        cv2.circle(result, center, radius, (0, 255, 0), 2)
        cv2.circle(result, center, 5, (0, 0, 255), -1)
        cv2.putText(result, "CSI Camera OK!", (30, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # ==========================================

        # 編碼為 JPEG
        ret, buffer = cv2.imencode('.jpg', result)
        frame_bytes = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    return render_template_string('''
        <html>
            <head><title>Pi CSI Stream</title></head>
            <body>
                <h1>Raspberry Pi CSI Camera Live</h1>
                <img src="/video_feed" width="640">
            </body>
        </html>
    ''')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)