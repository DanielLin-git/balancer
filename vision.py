import cv2
import time
import numpy as np
 
class VISION():
    def __init__(self):
        self.prev_time = None
        self.prev_center = None

        self.fps = 0.0

        # 色彩空間轉換 (RGB 轉 BGR 給 OpenCV 辨識用)
        self.lower = np.array([5, 100, 100])
        self.upper = np.array([25, 255, 255])
        self.kernel = np.ones((5, 5), np.uint8)

        self.min_area = 100

        self.ball_diameter = 40.0

    def recognition(self,frame):

        # 4. 計算 時間差）
        current_time = time.time()

        if self.prev_time is None:
            dt = None
        else:
            dt = current_time - self.prev_time

        self.prev_time = current_time

        #計算fps
        if dt is not None and dt > 0:
            instant_fps = 1.0 / dt
            if self.fps == 0:
                self.fps = instant_fps
            else:
                self.fps = 0.9 * self.fps + 0.1 * instant_fps

        frame = cv2.flip(frame, 1)
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        HSV_image = cv2.cvtColor(frame_bgr, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(HSV_image , self.lower, self.upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        # 3. 尋找輪廓 (比 Hough 快上好幾倍)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        ball = None
        center = None

        if contours:
            # 找出最大面積的輪廓（假設畫面中最大的目標就是球）
            c = max(contours, key=cv2.contourArea)
            
            # 確保面積大於一個微小閾值，避免雜訊誤判
            if cv2.contourArea(c) > self.min_area:
                # 計算最小外接圓
                ((x, y), radius) = cv2.minEnclosingCircle(c)

                center = (int(x), int(y))
                radius = int(radius)

                v_x = 0.0
                v_y = 0.0
                if self.prev_center is not None and dt>0:
                    # 像素距離換算
                    dx = (center[0] - self.prev_center[0])
                    dy = (self.prev_center[1] - center[1])
                    
                    scale = self.ball_diameter / (2.0 * radius)

                    v_x = dx / dt * scale
                    v_y =  dy / dt * scale 

                    ball = {
                        "x": x - frame.shape[1] / 2,
                        "y": frame.shape[0] / 2 - y,
                        "vx": v_x,
                        "vy": v_y,
                        "radius": radius,
                    }

                    cv2.circle(
                        frame_bgr, center, int(radius),
                        (0, 255, 0), 2
                    )

                    cv2.circle(
                        frame_bgr, center, 3, (0, 0, 255), -1
                    )

        self.prev_center = center

        height, width = frame_bgr.shape[:2]

        cv2.circle(
            frame_bgr, (width // 2, height // 2),
            3, (0, 255, 0), -1
        )

        cv2.putText(
            frame_bgr,
            f"FPS: {self.fps:.1f}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (0, 255, 0), 2
        )

        return frame_bgr, ball