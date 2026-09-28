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
        
        # 色彩空間轉換 (RGB 轉 BGR 給 OpenCV 辨識用)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # ==========================================
        # 在這裡加入你的 OpenCV 影像辨識與處理邏輯
        cv2.putText(frame, "CSI Camera OK!", (30, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # ==========================================

        # 編碼為 JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
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