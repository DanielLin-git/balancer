import time
import queue
import threading
import serial
import numpy as np
import cv2
import logging
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2

logging.basicConfig(level=logging.INFO, filename='log.log', filemode='a', format='%(asctime)s - %(levelname)s : %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p')

app = Flask(__name__)

# 開啟樹莓派的硬體序列埠 uart 通訊
ser = serial.Serial(port="/dev/serial0", baudrate=115200, timeout=0.01)

# 初始化 Picamera2
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (320, 240), "format": "RGB888"}))
picam2.start()

# 用於執行緒之間安全共享最新畫面的全域變數
output_frame = None
frame_lock = threading.Lock()


'''
class controller():
    def __init__(self):
        #ball properties
        self.ball_x
        self.ball_y
        self.ball_vx
        self.ball_vy
        
        #motor pwm
        self.motor_a_ms
        self.motor_b_ms
        self.motor_c_ms
        #control properties
        self.target_x
        self.target_y
        
        self.kp
        self.ki
        self.kd

    def controller()
'''
    
def send_servo_command(servo_id: int, microsecond: int):
  """將馬達編號與角度打包成自定義封包並發送"""
  header = b"\xAA\x55"  # 1. 標頭
  cmd = 0x10  # 2. 指令：控制伺服馬達
  length = 3  # 3. 長度：後面有 2 個位元組 (ID + microsecond)

  ms_high= (microsecond >> 8) & 0xFF
  ms_low = microsecond & 0xFF

  # 4. 準備 Payload
  payload = bytes([servo_id, ms_high, ms_low])

  # 5. 計算檢查碼 (對 Cmd、Length、Payload 做 XOR)
  chk = cmd ^ length ^ servo_id ^ ms_high ^ ms_low

  # 6. 組裝完整封包
  packet = header + bytes([cmd, length]) + payload + bytes([chk])

  # 7. 透過 UART 發送
  ser.write(packet)
  '''
  logging.info(
      f"[Pi 傳送] 馬達 ID: {servo_id} -> 毫秒{microsecond}° (封包長度:"
      f" {len(packet)} bytes)"
  )
  '''

def gen_frames():
    global output_frame
    prev_time = time.time()
    prev_center = None
    center = None
    radius = None
    fps = 0.0
    dim_ratio = 0.0

    # 色彩空間轉換 (RGB 轉 BGR 給 OpenCV 辨識用)
    lower = np.array([5, 100, 100])
    upper = np.array([25, 255, 255])

    while True:
        # 抓取相機影像陣列
        frame = picam2.capture_array()
        if frame is None:
            break

        # 4. 計算 時間差）
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        frame = cv2.flip(frame, 1)
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

                dim_ratio = 40/(2*radius)
                if prev_center is not None and dt>0:
                    # 像素距離換算 (pixel/sec)
                    pixel_distance = np.sqrt((center[0] - prev_center[0])**2 + (center[1] - prev_center[1])**2)
                    speed = pixel_distance / dt *dim_ratio
                    dx = (center[0] - prev_center[0])
                    dy = (center[1] - prev_center[1])
                    v_x = dx/dt*dim_ratio
                    v_y =  dy/dt*dim_ratio
                    #print(f"速度: {speed:.2f} px/s, 位置: {center}")
                    if abs(dx) >5 or abs(dy)>5:
                        logging.info(f"速度:{speed:.2f}, v_x:{v_x:.2f}, v_y:{v_y:.2f}, 位置:{x-160,y-120}, 半徑:{radius}")
                    
                    '''
                    #計算fps
                    instant_fps = 1 / dt
                    # 簡單的指數平滑，讓 FPS 數值穩定一點，不會亂跳
                    fps = (fps * 0.9) + (instant_fps * 0.1) if fps > 0 else instant_fps
                    logging.info(f"fps:{fps:.2f}")
                    '''

                prev_center = center
        
            #pid controller
            pwm1, pwm2, pwm3 = 1000, 1000, 1000

            # 發送給 ESP32 (非阻塞、高速)
            send_servo_command(1, pwm1)
            send_servo_command(2, pwm2)
            send_servo_command(3, pwm3)


        result = cv2.bitwise_and(frame, frame, mask=mask)
        # ==========================================
        # 在這裡加入你的 OpenCV 影像辨識與處理邏輯
        cv2.circle(result, center, radius, (0, 255, 0), 2)
        cv2.circle(result, center, 3, (0, 0, 255), -1)
        cv2.circle(result, (160,120), 2, (0, 255,0 ), -1)
        cv2.putText(result, "CSI Camera OK!", (30, 30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # ==========================================

        with frame_lock:
            output_frame = result.copy()
            
def generate_mjpeg():
  global output_frame
  while True:
    with frame_lock:
      if output_frame is None:
        continue
      # 壓縮為 JPEG，Q值設 60 即可兼顧流暢度與頻寬
      (flag, encodedImage) = cv2.imencode(".jpg", output_frame)
      if not flag:
        continue
      frame_bytes = encodedImage.tobytes()

    yield (
        b"--frame\r\n"
        b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
    )
    time.sleep(0.03)  # 限制網頁更新頻率約 30 FPS，釋放 CPU 給控制迴圈

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

@app.route("/video_feed")
def video_feed():
  return Response(
      generate_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame"
  )

if __name__ == '__main__':
    t = threading.Thread(target=gen_frames, daemon=True)
    t.start()

    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)