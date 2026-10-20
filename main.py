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

            error_x, error_y, output_x, output_y, theta, phi, motor_angles = controller.update(ball, dt, uart)
            
            logging.info(
                "Ball x=%.2f y=%.2f vx=%.2f vy=%.2f radius=%.2f error_x =%.2f error_y=%.2f output_x=%.2f output_y=%.2f theta=%.2f phi=%.2f motor_angles1=%.2f motor_angles2=%.2f motor_angles3=%.2f",
                ball["x"], ball["y"],
                ball["vx"], ball["vy"],
                ball["radius"],
                error_x, error_y, 
                output_x, output_y,
                theta, phi,
                motor_angles[0],motor_angles[1],motor_angles[2]
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