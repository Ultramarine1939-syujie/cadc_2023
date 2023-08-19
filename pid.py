import numpy as np

class PID():
    def __init__(self,Kp= 0.01,Ki= 0.001,Kd= 0.0) -> None:
        self.Kp  = Kp
        self.Ki  = Ki
        self.Kd  = Kd
        self.pos_previous_error = 0.0
        self.pos_integral = 0.0
        pass


    def calculate_pos_pid(self,target_pos,now_position):
        error = np.subtract(target_pos, now_position)
        self.pos_integral = np.add(self.pos_integral, error)
        derivative = np.subtract(error, self.pos_previous_error)
        self.pos_integral =  np.clip(self.pos_integral, -5.0, 5.0)
        control_signal = np.multiply(self.Kp, error) + np.multiply(self.Ki, self.pos_integral) + np.multiply(self.Kd, derivative)
        self.pos_previous_error = error
        return control_signal,error


