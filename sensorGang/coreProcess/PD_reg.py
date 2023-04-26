import time


class PDcontroller:

    def __init__(self, Kp, Kd):
        self.Kp = Kp
        self.Kd = Kd
        self.derivative_term = 0
        self.last_error = None
        self.last_time = 0
        self.steering = 0

    def get_control(self, center_offset):
        print("Kd : {}".format(self.Kd))
        print("Kp : {}".format(self.Kp))

        error = center_offset
        if self.last_error is not None:
            self.derivative_term = self.Kd * \
                (error-self.last_error) / (time.time()-self.last_time)
            self.last_time = time.time()
        self.PD_value = self.Kp*error + self.derivative_term
        self.last_error = error
        self.update_steering()
        return self.steering

    def update_steering(self):
        if abs(self.PD_value) > 10:
            if self.PD_value < 0:
                self.steering = (self.PD_value * 0.8)
            elif self.PD_value > 0:

                self.steering = self.PD_value
        else:
            self.steering = 0

    def updateKp(self, Kp):
        self.Kp = Kp

    def updateKd(self, Kd):
        self.Kp = Kd
