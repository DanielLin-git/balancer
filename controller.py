import math
import logging
from uart import UART

class CONTROLLER:
    def __init__(self):

        self.L0 = 69.75   # 圓心到馬達軸的距離
        self.L1 = 25.0   # 連桿1的長度 (下連桿)
        self.L2 = 100.0  # 連桿2的長度 (上連桿)
        self.L3 = 90.0   # 圓心到連桿二上方魚眼的距離 (頂板連接半徑)
        

        self.pz_ini = 99.89


        self.target_x = 0.0
        self.target_y = 0.0
        

        self.kp = 0.05
        self.ki = 0.0
        self.kd = 0.003

        self.integral_x = 0.0
        self.integral_y = 0.0


        self.pwm_min = 600
        self.pwm_max = 2000

    def motor_reset(self, uart: UART):
        self.send_angle_to_motor(uart, 1, 180)
        self.send_angle_to_motor(uart, 2, 180)
        self.send_angle_to_motor(uart, 3, 180)



    def motor_start(self, uart: UART):
        
        self.send_angle_to_motor(uart, 1, 180)
        self.send_angle_to_motor(uart, 2, 180)
        self.send_angle_to_motor(uart, 3, 180)


    def angle_to_pwm(self, angle_deg: float) -> int:

        angle_deg = max(0.0, min(180.0, angle_deg))
        pwm = self.pwm_min + (angle_deg / 180.0) * (self.pwm_max - self.pwm_min)
        return int(pwm)

    def send_angle_to_motor(self, uart: UART, servo_id: int, angle_deg: float):

        pwm = self.angle_to_pwm(angle_deg)
        uart.send_servo_command(servo_id, pwm)

    def kinema_inv(self, theta, phi, pz):
        """
        Inverse kinematics for a 3-actuator, 2-link platform.

        Parameters
        ----------
        theta : float
            Platform tilt about x-axis, in radians.
        phi : float
            Platform tilt about y-axis, in radians.
        pz : float
            Platform center height, in mm.

        Returns
        -------
        tuple
            (servo1, servo2, servo3), in degrees.

        Servo angle convention:
            180 deg = horizontal, pointing inward
            90 deg = vertical, pointing upward
        """

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


            """
            if not angle_min <= solution <= angle_max:
                raise ValueError("Servo angle outside safe range.")
            """
            solutions.append(solution)

        return tuple(solutions)

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

        tilt_gain_x = 0.015
        tilt_gain_y = 0.015

        theta = -output_y * tilt_gain_x
        phi = output_x * tilt_gain_y

        try:
            motor_angles = self.kinema_inv(
                theta, phi, self.pz_ini
            )

            for servo_id, angle in enumerate(motor_angles, start=1):
                self.send_angle_to_motor(
                    uart, servo_id, angle
                )

        except ValueError as e:
            logging.warning("IK failed: %s", e)




        motor_angles = self.kinema_inv(theta, phi, self.pz_ini)
        

        self.send_angle_to_motor(uart, 1, motor_angles[0])
        self.send_angle_to_motor(uart, 2, motor_angles[1])
        self.send_angle_to_motor(uart, 3, motor_angles[2])
        

        return error_x, error_y,output_x, output_y, theta, phi, motor_angles