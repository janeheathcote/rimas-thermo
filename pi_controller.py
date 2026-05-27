"""
Simple PI controller for temperature control.
PI = proportional term, integral term. (no derivative term)
"""

class PIController:
    def __init__(
        self,
        kp: float,
        ki: float,
        setpoint: float,
        u_min: float = 0.0,
        u_max: float = 1.0,
        integral_min: float = -1000.0,
        integral_max: float = 1000.0,
    ) -> None:
      
      
        self.kp = kp                # proportional gain
        self.ki = ki                # integral gain
        self.setpoint = setpoint    # target value

        self.u_min = u_min          # min/max allowed controller output
        self.u_max = u_max

        self.integral_min = integral_min    # min/max allowed integral term
        self.integral_max = integral_max

        self.integral = 0.0
        self._last_error = 0.0

    def reset(self) -> None:
        """
        Reset controller state (AKA restart the loop)
        """
        self.integral = 0.0
        self._last_error = 0.0
        

    def update(self, measured_value: float, dt: float) -> float:
        """
        Update the controller using the latest measured value.
        """
        
        # current control error
        error = self.setpoint - measured_value
        self._last_error = error

        # integral term
        self.integral += error * dt

        # apply min/max for integral (this is to minimize "windup")
        self.integral = max(self.integral_min, min(self.integral, self.integral_max))

        # PI output
        u = self.kp * error + self.ki * self.integral

        # apply min/max for final output
        u = max(self.u_min, min(self.u_max, u))
        return u

    @property
    def last_error(self) -> float:
        """
        Return the most recent error value.
        """
        return self._last_error

