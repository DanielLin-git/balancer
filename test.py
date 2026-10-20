
import math


class CONTROLLER:
    def __init__(self):
        self.L0 = 69.75
        self.L1 = 25.0
        self.L2 = 100.0
        self.L3 = 90.0

        # 上平台中心高度；零傾角時等於魚眼接點高度
        self.pz_ini = 99.89

        # 馬達方位角
        self.motor_azimuths = [270.0, 30.0, 150.0]

        # 伺服機械角度限制
        self.angle_min = 0.0
        self.angle_max = 180.0

        # 延續上一週期的解支
        self.last_angles = [180.0, 180.0, 180.0]

    @staticmethod
    def rotation_matrix(roll_deg, pitch_deg):
        """旋轉順序 R = Ry(pitch) @ Rx(roll)。"""
        r = math.radians(roll_deg)
        p = math.radians(pitch_deg)

        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)

        return [
            [cp, sp * sr, sp * cr],
            [0.0, cr, -sr],
            [-sp, cp * sr, cp * cr],
        ]

    def top_joint(self, azimuth_deg, roll_deg,
                  pitch_deg, center_height):
        """計算上平台魚眼接點的世界座標。"""
        g = math.radians(azimuth_deg)

        # 魚眼接點在上平台局部座標中的位置
        q = [
            self.L3 * math.cos(g),
            self.L3 * math.sin(g),
            0.0,
        ]

        R = self.rotation_matrix(roll_deg, pitch_deg)

        p = [
            sum(R[i][j] * q[j] for j in range(3))
            for i in range(3)
        ]

        p[2] += center_height
        return p

    def solve_leg(self, point, azimuth_deg, previous_angle):
        """單腿逆運動學；回傳最接近上一個角度的可行解。"""
        g = math.radians(azimuth_deg)
        px, py, pz = point

        # 將魚眼位置投影到馬達的徑向與切向
        rho = px * math.cos(g) + py * math.sin(g)
        tau = -px * math.sin(g) + py * math.cos(g)

        R = rho - self.L0
        z = pz

        # R*cos(alpha) - z*sin(alpha) = K
        K = (
            self.L2**2
            - self.L1**2
            - R**2
            - z**2
            - tau**2
        ) / (2.0 * self.L1)

        M = math.hypot(R, z)

        if M < 1e-12:
            raise ValueError("IK 奇異位置：M 接近零")

        ratio = K / M

        # 不可達姿態不可直接將判別式截成零
        tolerance = 1e-9
        if ratio < -1.0 - tolerance or ratio > 1.0 + tolerance:
            raise ValueError(
                f"不可達姿態：馬達方位角={azimuth_deg}, "
                f"K/M={ratio:.6f}"
            )

        ratio = max(-1.0, min(1.0, ratio))

        beta = math.acos(ratio)
        delta = math.atan2(z, R)

        # 兩個候選解
        candidates_rad = [
            beta - delta,
            -beta - delta,
        ]

        # 轉成 0~180 度內的等效伺服角度
        candidates = []
        for a in candidates_rad:
            deg = math.degrees(a)

            for k in range(-2, 3):
                angle = deg + 360.0 * k

                if self.angle_min <= angle <= self.angle_max:
                    candidates.append(angle)

        if not candidates:
            raise ValueError(
                f"沒有符合伺服角度限制的解："
                f"馬達方位角={azimuth_deg}"
            )

        return min(
            candidates,
            key=lambda a: abs(a - previous_angle)
        )

    def kinema_inv(self, roll_deg, pitch_deg, pz=None):
        """
        輸入平台 Roll / Pitch，回傳三顆伺服角度。
        pz 是上平台中心高度，不是傾斜後每個魚眼的高度。
        """
        if pz is None:
            pz = self.pz_ini

        # 先計算全部解，避免中途失敗時只更新部分狀態
        angles = []

        for i, azimuth in enumerate(self.motor_azimuths):
            point = self.top_joint(
                azimuth,
                roll_deg,
                pitch_deg,
                pz
            )

            angle = self.solve_leg(
                point,
                azimuth,
                self.last_angles[i]
            )

            angles.append(angle)

        self.last_angles = angles[:]
        return angles

    def check_ik(self, roll_deg, pitch_deg, pz=None):
        if pz is None:
            pz = self.pz_ini

        angles = self.kinema_inv(roll_deg, pitch_deg, pz)

        Rmat = self.rotation_matrix(roll_deg, pitch_deg)

        for i, (azimuth, alpha_deg) in enumerate(
            zip(self.motor_azimuths, angles)
        ):
            g = math.radians(azimuth)
            a = math.radians(alpha_deg)

            # 馬達軸心
            B = [
                self.L0 * math.cos(g),
                self.L0 * math.sin(g),
                0.0
            ]

            # 連桿 1 的末端
            E = [
                B[0] - self.L1 * math.cos(a) * math.cos(g),
                B[1] - self.L1 * math.cos(a) * math.sin(g),
                self.L1 * math.sin(a)
            ]

            # 上平台魚眼接點
            P = self.top_joint(
                azimuth, roll_deg, pitch_deg, pz
            )

            length = math.sqrt(sum(
                (P[j] - E[j])**2 for j in range(3)
            ))

            print(
                f"Motor {i+1}: angle={alpha_deg:.4f} deg, "
                f"L2_actual={length:.6f} mm, "
                f"error={length-self.L2:.6f} mm"
            )

        return angles


if __name__ == "__main__":
    controller = CONTROLLER()
    """
    # 測試 1：零傾角
    angles = controller.kinema_inv(
        roll_deg=0.0,
        pitch_deg=0.0
    )

    print("零傾角：", angles)

    # 測試 2：平台微幅傾斜
    angles = controller.kinema_inv(
        roll_deg=2.0,
        pitch_deg=2.0
    )

    print("Roll=2°, Pitch=2°：", angles)
    """
    controller.check_ik(0.0, 0.0)
    controller.check_ik(2.0, 2.0)