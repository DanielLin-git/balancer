
import time
import threading
import logging

from camera import CAMERA
from vision import VISION
from controller import CONTROLLER
from uart import UART
from web_stream import WEBSTREAM


def create_logger(name, filename):
    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)
    logger.propagate = False

    if not logger.handlers:
        handler = logging.FileHandler(
            filename,
            mode="a",
            encoding="utf-8"
        )

        handler.setFormatter(logging.Formatter(
            "%(asctime)s - %(levelname)s : %(message)s"
        ))

        logger.addHandler(handler)

    return logger


control_logger = create_logger(
    "control",
    "control.log"
)

params_logger = create_logger(
    "params",
    "params.log"
)


def main():
    camera = CAMERA()
    vision = VISION()
    controller = CONTROLLER()
    uart = UART()

    # 傳入正在執行的 controller，而不是另建一個控制器
    web_stream = WEBSTREAM(
        controller=controller,
        host="0.0.0.0",
        port=5000,
    )

    stop_event = threading.Event()

    camera_ready = False
    uart_ready = False
    worker = None

    def process_frames():
        previous_time = time.monotonic()
        last_log_time = 0.0

        while not stop_event.is_set():
            try:
                frame = camera.get_frame()

                if frame is None:
                    time.sleep(0.005)
                    continue

                now = time.monotonic()
                dt = now - previous_time
                previous_time = now

                result, ball = vision.recognition(frame)

                if ball is not None and dt > 0:
                    state = controller.update(ball, dt, uart)

                    # IK 無解時不解包，避免執行緒中斷
                    if state is not None:
                        (
                            error_x,
                            error_y,
                            output_x,
                            output_y,
                            theta,
                            phi,
                            motor_angles,
                        ) = state

                        # 限制日誌頻率，避免 logging 影響控制迴圈
                        if now - last_log_time >= 0.03:
                            control_logger.info(
                                "Ball x=%.2f y=%.2f "
                                "vx=%.2f vy=%.2f "
                                "ex=%.2f ey=%.2f "
                                "out_x=%.2f out_y=%.2f "
                                "theta=%.4f phi=%.4f "
                                "m1=%.2f m2=%.2f m3=%.2f dt=%.4f",
                                ball["x"],
                                ball["y"],
                                ball["vx"],
                                ball["vy"],
                                error_x,
                                error_y,
                                output_x,
                                output_y,
                                theta,
                                phi,
                                motor_angles[0],
                                motor_angles[1],
                                motor_angles[2],
                                dt,
                            )

                            last_log_time = now

                # 即使沒有偵測到球，也繼續更新網頁影像
                if result is not None:
                    web_stream.update_frame(result)

            except Exception:
                logging.exception("process_frames 發生錯誤")
                time.sleep(0.02)

    try:
        camera.camera_setup()
        camera_ready = True

        uart.uart_setup()
        uart_ready = True

        # 先將伺服移至目前設定的起始位置
        controller.motor_start(uart)

        worker = threading.Thread(
            target=process_frames,
            name="control-worker",
            daemon=True,
        )
        worker.start()

        logging.info("Flask server starting on port 5000")

        # Flask 在主執行緒運作
        # 重新整理瀏覽器不會重新執行這段初始化
        web_stream.run()

    except KeyboardInterrupt:
        logging.info("收到停止訊號")

    except Exception:
        logging.exception("主程式發生錯誤")
        raise

    finally:
        stop_event.set()

        if worker is not None:
            worker.join(timeout=2.0)

        # 關閉前將伺服移至預設角度
        if uart_ready:
            try:
                controller.motor_reset(uart)
            except Exception:
                logging.exception("伺服重設失敗")

        if camera_ready:
            try:
                camera.close()
            except Exception:
                logging.exception("相機關閉失敗")

        if uart_ready:
            try:
                uart.close()
            except Exception:
                logging.exception("UART 關閉失敗")

        logging.info("程式已結束")


if __name__ == "__main__":
    main()
