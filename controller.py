
import math
import threading
from uart import UART


class CONTROLLER:
    def __init__(self):
        self.L0 = 69.75
        self.L1 = 25.0
        self.L2 = 100.0
        self.L3 = 90.0

        self.pz_ini = 112.47

        self.pwm_min = 700
        self.pwm_max = 2100

        self.target_x = 0.0
        self.target_y = 0.0

        # 所有可由網頁調整的參數
        self._params = {
            "kp": 0.03,
            "ki": 0.0,
            "kd": 0.006,
            "tilt_gain_x": 0.05,
            "tilt_gain_y": 0.05,
            "theta_limit": 0.08,
            "phi_limit": 0.08,
        }

        # 保護參數的讀寫，避免 Flask 和控制執行緒競爭
        self._params_lock = threading.Lock()

        self.integral_x = 0.0
        self.integral_y = 0.0

        self._integral_limit = 1000.0

        # 保存最近一次成功的控制結果
        self._state_lock = threading.Lock()
        self._last_state = {
            "error_x": 0.0,
            "error_y": 0.0,
            "output_x": 0.0,
            "output_y": 0.0,
            "theta": 0.0,
            "phi": 0.0,
            "motor_angles": [150.0, 150.0, 150.0],
        }

    def get_params(self):
        """取得目前控制參數的快照。"""
        with self._params_lock:
            return dict(self._params)

    def set_params(self, updates):
        """
        僅接受白名單中的參數。
        驗證全部通過後才一次更新，避免部分更新。
        """
        bounds = {
            "kp": (0.0, 0.5),
            "ki": (0.0, 0.1),
            "kd": (0.0, 0.1),
            "tilt_gain_x": (0.0, 0.2),
            "tilt_gain_y": (0.0, 0.2),
            "theta_limit": (0.005, 0.15),
            "phi_limit": (0.005, 0.15),
        }

        if not isinstance(updates, dict) or not updates:
            raise ValueError("參數必須是非空 JSON 物件")

        validated = {}

        for key, value in updates.items():
            if key not in bounds:
                raise ValueError(f"不允許修改參數：{key}")

            if isinstance(value, bool):
                raise ValueError(f"{key} 必須是數值")

            try:
                value = float(value)
            except (TypeError, ValueError):
                raise ValueError(f"{key} 必須是數值")

            if not math.isfinite(value):
                raise ValueError(f"{key} 必須是有限數值")

            low, high = bounds[key]

            if not low <= value <= high:
                raise ValueError(
                    f"{key} 必須介於 {low} 與 {high}"
                )

            validated[key] = value

        with self._params_lock:
            self._params.update(validated)

        return self.get_params()

    def get_state(self):
        with self._state_lock:
            return dict(self._last_state)

    def motor_reset(self, uart: UART):
        for servo_id in (1, 2, 3):
            self.send_angle_to_motor(uart, servo_id, 150.0)

    def motor_start(self, uart: UART):
        for servo_id in (1, 2, 3):
            self.send_angle_to_motor(uart, servo_id, 150.0)

    def angle_to_pwm(self, angle_deg: float) -> int:
        angle_deg = max(0.0, min(180.0, angle_deg))
        pwm = self.pwm_min + (
            angle_deg / 180.0
        ) * (self.pwm_max - self.pwm_min)
        return int(pwm)

    def send_angle_to_motor(
        self, uart: UART, servo_id: int, angle_deg: float
    ):
        pwm = self.angle_to_pwm(angle_deg)
        uart.send_servo_command(servo_id, pwm)

    def kinema_inv(self, theta, phi, pz):
        l0 = self.L0
        l1 = self.L1
        l2 = self.L2
        l3 = self.L3

        alphas = [
            math.radians(270.0),
            math.radians(30.0),
            math.radians(150.0),
        ]

        ct = math.cos(theta)
        st = math.sin(theta)
        cp = math.cos(phi)
        sp = math.sin(phi)

        # 保留目前使用的旋轉矩陣
        R = [
            [cp,       0.0,  sp],
            [st * sp,  ct,  -st * cp],
            [-ct * sp, st,   ct * cp],
        ]

        P = [0.0, 0.0, pz]
        motor_angles = []

        for alpha in alphas:
            ca = math.cos(alpha)
            sa = math.sin(alpha)

            r = [
                l3 * ca,
                l3 * sa,
                0.0,
            ]

            C = [
                P[j] + sum(
                    R[j][k] * r[k] for k in range(3)
                )
                for j in range(3)
            ]

            Cr = C[0] * ca + C[1] * sa
            Ct = -C[0] * sa + C[1] * ca
            Cz = C[2]

            d = Cr - l0

            K = (
                l2**2 - d**2 - Ct**2 - Cz**2 - l1**2
            ) / (2.0 * l1)

            rho = math.hypot(d, Cz)

            if rho < 1e-9:
                return None

            ratio = K / rho

            if abs(ratio) > 1.0 + 1e-9:
                return None

            ratio = max(-1.0, min(1.0, ratio))

            gamma = math.acos(ratio)
            beta = math.atan2(Cz, d)

            candidates = []

            for sign in (1.0, -1.0):
                base = sign * gamma - beta

                for k in range(-2, 3):
                    q = base + 2.0 * math.pi * k

                    if -1e-9 <= q <= math.pi + 1e-9:
                        candidates.append(
                            max(0.0, min(math.pi, q))
                        )

            if not candidates:
                return None

            # 維持你目前的分支選擇方式
            q = min(
                candidates,
                key=lambda x: abs(x - math.pi)
            )

            motor_angles.append(math.degrees(q))

        return motor_angles

    def update(self, ball, dt, uart: UART):
        if ball is None or dt <= 0:
            return None

        # 一次讀取完整參數快照
        params = self.get_params()

        ball_x = float(ball["x"])
        ball_y = float(ball["y"])
        ball_vx = float(ball["vx"])
        ball_vy = float(ball["vy"])

        error_x = self.target_x - ball_x
        error_y = self.target_y - ball_y

        # 僅在啟用積分項時累積，避免 ki=0 時不必要地累積
        if params["ki"] > 0.0:
            self.integral_x += error_x * dt
            self.integral_y += error_y * dt

            self.integral_x = max(
                -self._integral_limit,
                min(self._integral_limit, self.integral_x)
            )
            self.integral_y = max(
                -self._integral_limit,
                min(self._integral_limit, self.integral_y)
            )
        else:
            self.integral_x = 0.0
            self.integral_y = 0.0

        output_x = (params["kp"] * error_x+ params["ki"] * self.integral_x+ params["kd"] * ball_vx)

        output_y = (params["kp"] * error_y+ params["ki"] * self.integral_y+ params["kd"] * ball_vy)

        theta_raw = -output_y * params["tilt_gain_x"]
        phi_raw = output_x * params["tilt_gain_y"]

        theta = max(-params["theta_limit"],min(theta_raw, params["theta_limit"]))

        phi = max(-params["phi_limit"],min(phi_raw, params["phi_limit"]))

        motor_angles = self.kinema_inv(theta, phi, self.pz_ini)

        if motor_angles is None:
            return None

        # IK 成功後才送出命令
        for servo_id, angle in enumerate(motor_angles, start=1):
            self.send_angle_to_motor(uart, servo_id, angle)

        state = {
            "error_x": error_x,
            "error_y": error_y,
            "output_x": output_x,
            "output_y": output_y,
            "theta": theta,
            "phi": phi,
            "motor_angles": list(motor_angles),
        }

        with self._state_lock:
            self._last_state = state

        return (
            error_x, error_y,
            output_x, output_y,
            theta, phi, motor_angles
        )
