import cv2
import time
import numpy as np
 
class VISION():
    def __init__(self):
        self.prev_time = None
        self.prev_position = None

        # 新增：濾波後的球心（像素座標）
        self.filtered_position = None

        # 新增：位置濾波時間常數
        self.position_tau = 0.06

        self.fps = 0.0

        # 色彩空間轉換 (RGB 轉 BGR 給 OpenCV 辨識用)
        self.lower = np.array([5, 100, 140])
        self.upper = np.array([20, 255, 255])
        self.kernel = np.ones((5, 5), np.uint8)

        self.min_area = 100

        self.ball_diameter = 40.0


        self.max_velocity_dt = 0.15

    def recognition(self,frame):

        current_time = time.perf_counter()

        frame = cv2.flip(frame, 1)
        HSV_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(HSV_image , self.lower, self.upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        # 3. 尋找輪廓 (比 Hough 快上好幾倍)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        ball = None
        center = None

        if contours:
            c = max(contours, key=cv2.contourArea)

            if cv2.contourArea(c) > self.min_area:
                (x, y), radius = cv2.minEnclosingCircle(c)

                if radius > 0:
                    center = (float(x), float(y))
                    radius = float(radius)

        if center is not None:
            
            scale = self.ball_diameter / (2.0 * 36)

            if self.filtered_position is None or self.prev_time is None:
                self.filtered_position = center
                self.prev_position = center
                self.prev_time = current_time

            else:
                
                dt = current_time - self.prev_time

                if 0 < dt <= self.max_velocity_dt:
                    
                    
                    alpha = 1.0 - np.exp(-dt / self.position_tau)

                    old_fx, old_fy = self.filtered_position
                    raw_x, raw_y = center

                    filtered_x = old_fx + alpha * (raw_x - old_fx)
                    filtered_y = old_fy + alpha * (raw_y - old_fy)

                    self.filtered_position = (filtered_x, filtered_y)

                    # 使用濾波後的位置計算速度
                    prev_x, prev_y = self.prev_position

                    dx = filtered_x - prev_x
                    dy = prev_y - filtered_y

                    v_x = dx / dt * scale
                    v_y =  dy / dt * scale 
                    radius_mm = radius *scale

                    ball = {
                        "x": (filtered_x - frame.shape[1] / 2)*scale,
                        "y": (frame.shape[0] / 2 - filtered_y)*scale,
                        "vx": v_x,
                        "vy": v_y,
                        "radius": radius_mm,
                        "scale":scale
                    }

                    cv2.circle(
                        frame, (round(center[0]), round(center[1])),
                        round(radius),
                        (0, 255, 0), 2
                    )

                    cv2.circle(
                        frame, (round(center[0]), round(center[1])),
                        3, (0, 0, 255), -1
                    )

                    # 只有成功偵測到球時才更新歷史資料
                    self.prev_position = (filtered_x, filtered_y)
                    self.prev_time = current_time

            cv2.circle(
                frame, 
                (round(center[0]), round(center[1])),
                round(radius), (0, 255, 0), 2
            )

            cv2.circle(
                frame,
                (round(center[0]), round(center[1])),
                3, (0, 0, 255), -1
            )

        else:
            # 間隔過長：重新建立基準
            self.filtered_position = center
            self.prev_position = center
            self.prev_time = current_time

            self.vx_filtered = 0.0
            self.vy_filtered = 0.0



        height, width = frame.shape[:2]

        cv2.circle(
            frame, (width // 2, height // 2),
            3, (0, 255, 0), -1
        )

        cv2.putText(
            frame,
            f"FPS: {self.fps:.1f}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (0, 255, 0), 2
        )

        return frame, ball