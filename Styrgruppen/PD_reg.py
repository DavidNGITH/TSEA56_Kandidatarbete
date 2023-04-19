import time

class PDcontroller:


    def __init__(self,Kp,Kd):
        self.Kp = 100
        self.Kd = 1
        self.derivative_term = 0
        self.last_error = None
        self.last_time=0

    def get_control(self, center_offset,dt):
        error = center_offset
        self.last_error = error
        if self.last_error is not None:
            self.derivative_term = (error-self.last_error)/(time.time()-self.last_time)*self.Kd
            self.last_time=time.time()
        self.PD_value= self.Kp*error + self.derivative_term 

    def update_steering(self):
        if abs(self.PD_value) > 1:
            if self.PD_value < 0:
                self.steering += self.PD_value*5
            elif self.PD_value > 0:
                self.steering -= self.PD_value*5
        print(self.steering)

    
    def obstacle_control(self, obs_distance):
        

        if obs_distance < 10:
            self.breaking = 1
        else:
            self.breaking = 0
        
        




    




