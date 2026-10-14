import math
from uart import UART

class CONTROLLER:
    def __init__(self):
        # 機構與連桿尺寸 (單位: mm)
        self.L0 = 60.0   # 圓心到馬達軸的距離
        self.L1 = 25.0   # 連桿1的長度 (下連桿)
        self.L2 = 100.0  # 連桿2的長度 (上連桿)
        self.L3 = 90.0   # 圓心到連桿二上方魚眼的距離 (頂板連接半徑)
        
        # 初始高度
        self.pz_ini = 100.0

        # 控制目標位置 (畫面中心預設為 0, 0)
        self.target_x = 0.0
        self.target_y = 0.0
        
        # PID 參數 (可依實際測試微調)
        self.kp = 0.03
        self.ki = 0
        self.kd = 0

        # 積分項累積
        self.integral_x = 0.0
        self.integral_y = 0.0

        # PWM 映射範圍設定 (0度 = 500, 180度 = 2000)
        self.pwm_min = 500
        self.pwm_max = 2000

    def motor_reset(self, uart: UART):
        """將所有馬達歸零/重置位置"""
        self.send_angle_to_motor(uart, 1, 0.0)
        self.send_angle_to_motor(uart, 2, 0.0)
        self.send_angle_to_motor(uart, 3, 0.0)

    def motor_start(self, uart: UART):
        """啟動時將馬達推到初始平衡位置"""
        angles = self.kinema_inv(0.0, 0.0, self.pz_ini)
        self.send_angle_to_motor(uart, 1, angles[0])
        self.send_angle_to_motor(uart, 2, angles[1])
        self.send_angle_to_motor(uart, 3, angles[2])

    def angle_to_pwm(self, angle_deg: float) -> int:
        """將角度 (0~180度) 轉換為對應的 PWM 微秒數 (500~2000)"""
        # 限制角度在 0 到 180 之間
        angle_deg = max(0.0, min(180.0, angle_deg))
        pwm = self.pwm_min + (angle_deg / 180.0) * (self.pwm_max - self.pwm_min)
        return int(pwm)

    def send_angle_to_motor(self, uart: UART, servo_id: int, angle_deg: float):
        """計算 PWM 並透過 UART 發送給指定的馬達"""
        pwm = self.angle_to_pwm(angle_deg)
        uart.send_servo_command(servo_id, pwm)

    def kinema_inv(self, theta_platform: float, phi_platform: float, pz: float):
        """
        3-DOF Stewart 平臺逆運動學計算
        - theta_platform: 平台傾斜方位角 (度)
        - phi_platform: 平台傾斜幅度/傾角 (度)
        - pz: 平台高度 (mm)
        回傳: [theta_a, theta_b, theta_c] 三顆馬達的目標角度 (度)
        """
        L0, L1, L2, L3 = self.L0, self.L1, self.L2, self.L3

        # 將平台傾斜轉為頂板法向量 n = [x, y, z]
        phi_rad = math.radians(phi_platform)
        theta_rad = math.radians(theta_platform)
        
        z_n = math.cos(phi_rad)
        r_n = math.sin(phi_rad)
        nx = r_n * math.cos(theta_rad)
        ny = r_n * math.sin(theta_rad)
        nz = z_n

        # 參考基準高度 Pmz 導出
        A_base = (L0 + L1) / pz
        B_base = (pz**2 + L2**2 - (L0 + L1)**2 - L3**2) / (2 * pz)
        C_base = A_base**2 + 1
        D_base = 2 * (A_base * B_base - (L0 + L1))
        E_base = B_base**2 + (L0 + L1)**2 - L2**2
        
        disc_base = D_base**2 - 4 * C_base * E_base
        if disc_base < 0:
            disc_base = 0  # 防止浮點數開根號負數報錯
        pmz = math.sqrt(max(0, L2**2 - ((-D_base + math.sqrt(disc_base)) / (2 * C_base))**2 + 2*(L0+L1)*((-D_base + math.sqrt(disc_base)) / (2 * C_base)) - (L0+L1)**2))

        # --- 馬達 A (0度方向) ---
        a_m_z = pz + (L3 / math.sqrt(nx**2 + nz**2)) * (-nx) if (nx**2 + nz**2) > 0 else pz
        a_m_x = (L3 / math.sqrt(nx**2 + nz**2)) * nz if (nx**2 + nz**2) > 0 else 0
        
        A_a = (L0 - a_m_x) / a_m_z if a_m_z != 0 else 0
        B_a = (a_m_x**2 + a_m_z**2 - L2**2 - L0**2 + L1**2) / (2 * a_m_z) if a_m_z != 0 else 0
        C_a = A_a**2 + 1
        D_a = 2 * (A_a * B_a - L0)
        E_a = B_a**2 + L0**2 - L1**2
        
        disc_a = D_a**2 - 4 * C_a * E_a
        ax = (-D_a + math.sqrt(max(0, disc_a))) / (2 * C_a)
        az = math.sqrt(max(0, L1**2 - ax**2 + 2*L0*ax - L0**2))
        if a_m_z < pmz:
            az = -az
        theta_a = 90.0 - math.degrees(math.atan2(ax - L0, az))

        # --- 馬達 B (120度方向) ---
        denom_b = math.sqrt(nx**2 + 3*ny**2 + 4*nz**2 + 2*math.sqrt(3)*nx*ny)
        if denom_b > 0:
            b_m_x = (L3 / denom_b) * (-nz)
            b_m_y = (L3 / denom_b) * (-math.sqrt(3)*nz)
            b_m_z = pz + (L3 / denom_b) * (math.sqrt(3)*ny + nx)
        else:
            b_m_x, b_m_y, b_m_z = 0, 0, pz

        A_b = -(b_m_x + math.sqrt(3)*b_m_y + 2*L0) / b_m_z if b_m_z != 0 else 0
        B_b = (b_m_x**2 + b_m_y**2 + b_m_z**2 + L1**2 - L2**2 - L0**2) / (2 * b_m_z) if b_m_z != 0 else 0
        C_b = A_b**2 + 4
        D_b = 2 * A_b * B_b + 4 * L0
        E_b = B_b**2 + L0**2 - L1**2
        
        disc_b = D_b**2 - 4 * C_b * E_b
        bx = (-D_b - math.sqrt(max(0, disc_b))) / (2 * C_b)
        bz = math.sqrt(max(0, L1**2 - 4*bx**2 - 4*L0*bx - L0**2))
        if b_m_z < pmz:
            bz = -bz
        theta_b = 90.0 - math.degrees(math.atan2(math.sqrt(3*bx**2 + 3*bx**2) - L0, bz)) # 簡化半徑對應

        # --- 馬達 C (240度方向) ---
        denom_c = math.sqrt(nx**2 + 3*ny**2 + 4*nz**2 - 2*math.sqrt(3)*nx*ny)
        if denom_c > 0:
            c_m_x = (L3 / denom_c) * (-nz)
            c_m_y = (L3 / denom_c) * (math.sqrt(3)*nz)
            c_m_z = pz + (L3 / denom_c) * (-math.sqrt(3)*ny + nx)
        else:
            c_m_x, c_m_y, c_m_z = 0, 0, pz

        A_c = -(c_m_x - math.sqrt(3)*c_m_y + 2*L0) / c_m_z if c_m_z != 0 else 0
        B_c = (c_m_x**2 + c_m_y**2 + c_m_z**2 + L1**2 - L2**2 - L0**2) / (2 * c_m_z) if c_m_z != 0 else 0
        C_c = A_c**2 + 4
        D_c = 2 * A_c * B_c + 4 * L0
        E_c = B_c**2 + L0**2 - L1**2
        
        disc_c = D_c**2 - 4 * C_c * E_c
        cx = (-D_c - math.sqrt(max(0, disc_c))) / (2 * C_c)
        cz = math.sqrt(max(0, L1**2 - 4*cx**2 - 4*L0*cx - L0**2))
        if c_m_z < pmz:
            cz = -cz
        theta_c = 90.0 - math.degrees(math.atan2(math.sqrt(3*cx**2 + 3*cx**2) - L0, cz))

        return [theta_a, theta_b, theta_c]

    def update(self, ball, dt, uart: UART):
        """
        主更新迴圈：
        1. 接收球的位置與速度 (來自 vision)
        2. 透過 PID 計算外環平台的 Roll/Pitch 傾斜角度
        3. 透過逆運動學 (Inverse Kinematics) 將角度轉為三顆馬達角度
        4. 透過 UART 直接發送 PWM 控制指令給馬達
        """
        if ball is None or dt <= 0:
            return

        ball_x = ball["x"]
        ball_y = ball["y"]
        ball_vx = ball["vx"]
        ball_vy = ball["vy"]

        # 1. 計算位置誤差
        error_x = ball_x - self.target_x
        error_y = ball_y - self.target_y

        # 2. 積分項累積與微分項計算
        self.integral_x += error_x * dt
        self.integral_y += error_y * dt

        derivative_x = ball_vx  # 速度直接等於誤差的微分 (dx/dt)
        derivative_y = ball_vy

        # 3. PID 輸出計算 (得出直角座標系的力道/分量)
        output_x = self.kp * error_x + self.ki * self.integral_x + self.kd * derivative_x
        output_y = self.kp * error_y + self.ki * self.integral_y + self.kd * derivative_y

        # 4. 轉換為平台極座標姿態 (theta: 方位角, phi: 傾斜幅度)
        theta = math.degrees(math.atan2(output_y, output_x))
        if theta < 0:
            theta += 360.0

        # 將輸出大小對應到實際平台最大傾斜角限制 (例如最大限制 15 度)
        phi = math.sqrt(output_x**2 + output_y**2) * 1000.0  # 依實際增益調整縮放
        max_phi = 15.0
        if phi > max_phi:
            phi = max_phi

        # 5. 透過逆運動學計算三顆馬達的目標角度
        motor_angles = self.kinema_inv(theta, phi, self.pz_ini)

        # 6. 轉換為 PWM 並透過 UART 發送給 ESP32 控制三顆馬達
        self.send_angle_to_motor(uart, 1, motor_angles[0])
        self.send_angle_to_motor(uart, 2, motor_angles[1])
        self.send_angle_to_motor(uart, 3, motor_angles[2])

        return theta, phi