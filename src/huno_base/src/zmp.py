import numpy as np

class ZMPGaitGenerator:
    def __init__(self, step_time=0.6, step_length=0.08, com_height=0.25):
        self.step_time = step_time
        self.step_length = step_length
        self.com_height = com_height
        self.g = 9.81

    def generate(self, num_steps=10, dt=0.01):
        t = np.arange(0, num_steps * self.step_time, dt)
        omega = np.sqrt(self.g / self.com_height)

        x = np.zeros_like(t)
        x_dot = 0.0
        x_pos = 0.0

        zmp_x = np.zeros_like(t)

        for i in range(len(t)):
            step_idx = int(t[i] / self.step_time)
            zmp_x[i] = step_idx * self.step_length

            x_ddot = omega**2 * (x_pos - zmp_x[i])
            x_dot += x_ddot * dt
            x_pos += x_dot * dt
            x[i] = x_pos

        return t, x, zmp_x
