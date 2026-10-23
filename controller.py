import math
import logging
from uart import UART

class CONTROLLER:
    def __init__(self):

        self.L0 = 69.75   # 圓心到馬達軸的距離
        self.L1 = 25.0   # 連桿1的長度 (下連桿)
        self.L2 = 100.0  # 連桿2的長度 (上連桿)
        self.L3 = 90.0   # 圓心到連桿二上方魚眼的距離 (頂板連接半徑)
        

        self.pz_ini = 112.47


        self.target_x = 0.0
        self.target_y = 0.0
        

        self.kp = 0.03
        self.ki = 0.0
        self.kd = 0.003

        self.integral_x = 0.0
        self.integral_y = 0.0


        self.pwm_min = 600
        self.pwm_max = 2000

    def motor_reset(self, uart: UART):
        self.send_angle_to_motor(uart, 1, 150)
        self.send_angle_to_motor(uart, 2, 150)
        self.send_angle_to_motor(uart, 3, 150)



    def motor_start(self, uart: UART):
        
        self.send_angle_to_motor(uart, 1, 150)
        self.send_angle_to_motor(uart, 2, 150)
        self.send_angle_to_motor(uart, 3, 150)


    def angle_to_pwm(self, angle_deg: float) -> int:

        angle_deg = max(0.0, min(180.0, angle_deg))
        pwm = self.pwm_min + (angle_deg / 180.0) * (self.pwm_max - self.pwm_min)
        return int(pwm)

    def send_angle_to_motor(self, uart: UART, servo_id: int, angle_deg: float):

        pwm = self.angle_to_pwm(angle_deg)
        uart.send_servo_command(servo_id, pwm)
    

    """
    def kinema_inv(self, theta, phi, pz):
        
        Inverse kinematics for a 3-actuator, 2-link platform.

        Parameters
        
        theta : float
            Platform tilt about x-axis, in radians.
        phi : float
            Platform tilt about y-axis, in radians.
        pz : float
            Platform center height, in mm.

        Returns
        
        tuple
            (servo1, servo2, servo3), in degrees.
        


        motor_positions = [270.0, 30.0, 150.0]

        angle_min = 90.0
        angle_max = 180.0

        solutions = []

        for alpha_deg in motor_positions:

            alpha = math.radians(alpha_deg)

            # Upper joint coordinates
            xt = self.L3 * math.cos(alpha)
            yt = self.L3 * math.sin(alpha)

            # Height of upper joint after platform tilt
            zt = pz + yt * math.tan(theta) \
                    - xt * math.tan(phi)

            # Distance constraint:
            # distance(lower-link endpoint, upper joint) = L2
            def constraint(beta_deg):

                beta = math.radians(beta_deg)

                # Radial position of lower-link endpoint
                rc = self.L0 + self.L1 * math.cos(beta)

                # Vertical position of lower-link endpoint
                zc = self.L1 * math.sin(beta)

                dx = self.L3 - rc
                dz = zt - zc

                return dx * dx + dz * dz - self.L2 ** 2

            # Search only within [90, 180].
            # Scan from 180 downward to prioritize the
            # solution closest to the initial servo angle.
            step = 0.25
            beta_prev = angle_max
            f_prev = constraint(beta_prev)

            solution = None

            while beta_prev - step >= angle_min:

                beta_next = beta_prev - step
                f_next = constraint(beta_next)

                if f_prev == 0.0:
                    solution = beta_prev
                    break

                if f_prev * f_next <= 0.0:

                    # Bisection method
                    lo = beta_next
                    hi = beta_prev
                    flo = constraint(lo)

                    for _ in range(40):

                        mid = (lo + hi) / 2.0
                        fm = constraint(mid)

                        if abs(fm) < 1e-8:
                            lo = mid
                            hi = mid
                            break

                        if flo * fm <= 0.0:
                            hi = mid
                        else:
                            lo = mid
                            flo = fm

                    solution = (lo + hi) / 2.0
                    break

                beta_prev = beta_next
                f_prev = f_next

            # Check the final endpoint
            if solution is None:
                if abs(constraint(angle_min)) < 1e-6:
                    solution = angle_min
                elif abs(constraint(angle_max)) < 1e-6:
                    solution = angle_max

            if solution is None:
                pass

                # At least one leg cannot reach its target
                # within the allowed servo range.
                raise ValueError(
                    f"No IK solution for motor at "
                    f"{alpha_deg:.0f} degrees. "
                    f"Check platform tilt and geometry."
                )


            if not angle_min <= solution <= angle_max:
                raise ValueError("Servo angle outside safe range.")

            solutions.append(solution)

        return tuple(solutions)
    """

    def kinema_inv(self, theta, phi, pz):


        l0 = self.L0
        l1 = self.L1
        l2 = self.L2
        l3 = self.L3

        # 三顆馬達的方位角，採世界座標逆時針為正
        alphas = [
            math.radians(270.0),
            math.radians(30.0),
            math.radians(150.0),
        ]

        # Roll 與 Pitch 的三角函數
        ct = math.cos(theta)
        st = math.sin(theta)
        cp = math.cos(phi)
        sp = math.sin(phi)

        # R = Rx(theta) @ Ry(phi)
        R = [
            [cp,       0.0,  sp],
            [st * sp,  ct,  -st * cp],
            [-ct * sp, st,   ct * cp],
        ]

        # 平台中心
        P = [0.0, 0.0, pz]

        motor_angles = []

        for alpha in alphas:

            ca = math.cos(alpha)
            sa = math.sin(alpha)

            # 上平台接點在平台局部座標系中的位置
            r = [
                l3 * ca,
                l3 * sa,
                0.0,
            ]

            # C = P + R @ r
            C = [
                P[j] + sum(R[j][k] * r[k] for k in range(3))
                for j in range(3)
            ]

            # 轉換到馬達的徑向、切向、垂直座標
            Cr = C[0] * ca + C[1] * sa
            Ct = -C[0] * sa + C[1] * ca
            Cz = C[2]

            d = Cr - l0

            # d*cos(q) - Cz*sin(q) = K
            K = (
                l2**2 - d**2 - Ct**2 - Cz**2 - l1**2
            ) / (2.0 * l1)

            rho = math.hypot(d, Cz)

            # 檢查是否存在實數解
            if rho < 1e-9:
                return None

            ratio = K / rho

            if abs(ratio) > 1.0 + 1e-9:
                return None

            # 避免浮點誤差使 acos 超出定義域
            ratio = max(-1.0, min(1.0, ratio))

            gamma = math.acos(ratio)
            beta = math.atan2(Cz, d)

            # 產生不同分支及其等價角度
            candidates = []

            for sign in (1.0, -1.0):
                base = sign * gamma - beta

                for k in range(-2, 3):
                    q = base + 2.0 * math.pi * k

                    if -1e-9 <= q <= math.pi + 1e-9:
                        candidates.append(
                            max(0.0, min(math.pi, q))
                        )
            """
            print(
                f"alpha={math.degrees(alpha):.1f}, "
                f"Cr={Cr:.4f}, Ct={Ct:.4f}, Cz={Cz:.4f}, "
                f"d={d:.4f}, K={K:.4f}, rho={rho:.4f}, "
                f"ratio={ratio:.6f}, "
                f"candidates={[round(math.degrees(q), 4) for q in candidates]}"
            )
            """
            if not candidates:
                return None

            # 優先選擇最接近水平朝外的 180 度解
            q = min(
                candidates,
                key=lambda x: abs(x - math.pi)
            )

            motor_angles.append(math.degrees(q))

        return motor_angles

    def update(self, ball, dt, uart: UART):

        if ball is None or dt <= 0:
            return

        ball_x = ball["x"]
        ball_y = ball["y"]
        ball_vx = ball["vx"]
        ball_vy = ball["vy"]

        error_x = self.target_x - ball_x
        error_y = self.target_y  - ball_y

        self.integral_x += error_x * dt
        self.integral_y += error_y * dt

        derivative_x = ball_vx  
        derivative_y = ball_vy


        output_x = self.kp * error_x + self.ki * self.integral_x - self.kd * derivative_x
        output_y = self.kp * error_y + self.ki * self.integral_y - self.kd * derivative_y

        tilt_gain_x = 0.05
        tilt_gain_y = 0.05

        theta = -output_y * tilt_gain_x
        phi = output_x * tilt_gain_y

        if theta >=0.08:
            theta =0.08
        elif theta<=-0.08:
            theta = -0.08
        
        if phi >=0.08:
            phi =0.08
        elif phi<=-0.08:
            phi = -0.08

        motor_angles = self.kinema_inv(theta, phi, self.pz_ini)
        

        self.send_angle_to_motor(uart, 1, motor_angles[0])
        self.send_angle_to_motor(uart, 2, motor_angles[1])
        self.send_angle_to_motor(uart, 3, motor_angles[2])
        

        return error_x, error_y,output_x, output_y, theta, phi, motor_angles