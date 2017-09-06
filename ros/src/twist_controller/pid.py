
INFINITY = float('inf')

class PID(object):
    """PID Controller."""

    def __init__(self, kp, ki, kd, min=-INFINITY, max=INFINITY):
        """Initializes the PID controller.

        Args:
            kp (float): gain for the proportional term
            ki (float): gain for the integral term
            kd (float): gain for the derivative term
            min (float): minimum output value
            max (float): maximum output value
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = min
        self.max = max
        self.int_val = 0.
        self.last_int_val = 0.
        self.last_error = 0.

    def reset_integrator(self):
        """Resets the integral error."""
        self.int_val = 0.
        self.last_int_val = 0.

    def revert_integrator(self):
        """Reverts the integral error to the previous value."""
        self.int_val = self.last_int_val

    def step(self, error, sample_time):
        """Runs one time step in the controller.

        Args:
            error (float): target setpoint
            sample_time (float): control period

        Returns:
            float: control setpoint
        """
        derivative = (error - self.last_error) / sample_time;

        val = self.kp * error + self.ki * self.int_val + self.kd * derivative;

        # If we are outside the safe area clamp to limits
        # and don't update the integrator
        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            # Update the integral error
            self.last_int_val = self.int_val
            self.int_val += error * sample_time;

        self.last_error = error
        return val

    def __str__(self):
        """Returns a string representation useful for debugging."""
        return "<PID Kp={:.2f} Ki={:.2f} Kd={:.2f} int={:.3f} val={:.3f}>".format(
            self.kp, self.ki, self.kd, self.int_val, self.last_error)
