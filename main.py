
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
            control_output = controller.update(ball, dt)

            if control_output is not None:
                # 暫時保留安全的測試值。
                # 後續再將 control_output 映射到三顆馬達。
                pwm1, pwm2, pwm3 = 1600, 1600, 1600

                uart.send_servo_command(1, pwm1)
                uart.send_servo_command(2, pwm2)
                uart.send_servo_command(3, pwm3)

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

    try:
        web_stream.run()
    finally:
        stop_event.set()
        worker.join(timeout=2.0)
        camera.close()
        uart.close()


if __name__ == "__main__":
    main()