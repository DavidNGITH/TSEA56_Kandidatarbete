"""PD Controller."""
import time


class PDcontroller:
    """Class for PD controller."""

    def __init__(self, Kp, Kd):
        """Init variables."""
        self.Kp = Kp
        self.Kd = Kd
        self.derivative_term = 0
        self.last_error = None
        self.last_time = 0
        self.steering = 0

    def get_control(self, center_offset):
        """Calculate steering."""
        error = center_offset
        if self.last_error is not None:
            self.derivative_term = (error-self.last_error) / \
                (time.time()-self.last_time)*self.Kd
            self.last_time = time.time()
        self.PD_value = self.Kp*error + self.derivative_term
        self.last_error = error
        self.update_steering()
        return self.steering

    def update_steering(self):
        """Check threshold."""
        if abs(self.PD_value) > 1:
            self.steering = self.PD_value
        else:
            self.steering = 0

    def updateKp(self, Kp):
        """Update Kp value."""
        self.Kp = Kp

    def updateKd(self, Kd):
        """Update Kd value."""
        self.Kp = Kd
