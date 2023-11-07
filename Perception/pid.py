class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.error_integral = 0
        self.previous_error = 0

    def calculate(self, error):
        # Proportional term
        P = self.Kp * error

        # Integral term
        self.error_integral += error
        I = self.Ki * self.error_integral

        # Derivative term
        D = self.Kd * (error - self.previous_error)
        self.previous_error = error

        # Total control signal
        control_signal = P + I + D

        return control_signal
