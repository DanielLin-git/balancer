import time
from picamera2 import Picamera2

class CAMERA():
    def __init__(self):
        self.picam2 = None
        self.frame = None

    def camera_setup(self):
        # 初始化 Picamera2
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_video_configuration(main={"size": (320, 240), "format": "RGB888"}))
        self.picam2.start()

    def get_frame(self):
        if self.picam2 is None:
            raise RuntimeError("Camera has not been initialized")
        # 抓取相機影像陣列
        self.frame = self.picam2.capture_array()

        return self.frame


    def close(self):
        if self.picam2 is not None:
            self.picam2.stop()
            self.picam2 = None