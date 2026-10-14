
import time
import threading
import logging

import cv2


from camera import CAMERA
from vision import VISION
from controller import CONTROLLER
from uart import UART
from web_stream import WEBSTREAM

logging.basicConfig(
    level=logging.INFO,
    filename="log.log",
    filemode="a",
    format="%(asctime)s - %(levelname)s : %(message)s"
)

#物件初始化
camera = CAMERA()
vision = VISION()
controller = CONTROLLER()
uart = UART()
web_stream = WEBSTREAM()

latest_jpeg = None

stop_event = threading.Event()


def process_frames():
    global latest_jpeg

    previous_time = time.monotonic()

    while not stop_event.is_set():
        frame = camera.get_frame()

        if frame is None:
            time.sleep(0.005)
            continue

        now = time.monotonic()
        dt = now - previous_time
        previous_time = now

        result, ball = vision.recognition(frame)

        if ball is not None and dt > 0:

            controller.update(ball, dt, uart)
            
            logging.info(
                "Ball x=%.2f y=%.2f vx=%.2f vy=%.2f",
                ball["x"], ball["y"],
                ball["vx"], ball["vy"]
            )


        web_stream.update_frame(result)


def main():
    camera.camera_setup()
    uart.uart_setup()
    

    worker = threading.Thread(
        target=process_frames,
        daemon=True
    )
    worker.start()

    controller.motor_start(uart)

    try:
        web_stream.run()
    finally:
        controller.motor_reset(uart)
        stop_event.set()
        worker.join(timeout=2.0)
        camera.close()
        uart.close()


if __name__ == "__main__":
    main()