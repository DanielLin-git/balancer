
class CONTROLLER():
    def __init__(self):
        #ball properties
        self.ball_x = None
        self.ball_y = None
        self.ball_vx = None
        self.ball_vy = None
        
        #motor pwm
        self.motor_a_ms  = None
        self.motor_b_ms  = None
        self.motor_c_ms  = None
        #control properties
        self.target_x = None
        self.target_y = None
        
        self.stage1_kp = None
        self.stage1_ki = None
        self.stage1_kd = None



    def update(self, ball, dt):
        return True
