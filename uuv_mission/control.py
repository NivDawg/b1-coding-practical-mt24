# Implementation of a simple PD controller for regulating the depth of an Unmanned Underwater Vehicle (UUV).


class Controller:

    """ A simple PD controller to regulate the depth of the UUV.
    Computes the control action
        u[t] = Kp * e[t] + Kd * (e[t] - e[t-1]) 
    where e[t] is the error (r[t] - y[t]) at time t."""

    def __init__(self, Kp: float = 0.15, Kd: float = 0.6, init_error: float = 0.0):

        """ Initialize the controller with given gains and initial error. """

        self.Kp = Kp    # Proportional gain
        self.Kd = Kd    # Derivative gain
        self.prev_error = init_error    # e[t-1]

    def __call__(self, observation: float, reference: float) -> float:
        """ Compute the control action based on the current observation and reference. """

        error = reference - observation     # e[t]
        derivative = error - self.prev_error    #  e[t] - e[t-1]
        self.prev_error = error # Update previous error for next iteration
        return self.Kp * error + self.Kd * derivative   # u[t] = Kp * e[t] + Kd * (e[t] - e[t-1])

