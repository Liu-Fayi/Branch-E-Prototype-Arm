import math

class IKSolver():
    def __init__(self, forearm_length, upperarm_length):
        self.r1 = forearm_length
        self.r2 = upperarm_length

    def solve(self, x: float, y: float) -> tuple:
        # Solve the inverse kinematics problem
        # and return the angles of the two joints
        t_avg = math.atan2(y, x)
        d = (self.r1**2 + self.r2**2 - x**2 - y**2) / (2 * self.r1 * self.r2)
        d = math.acos(d)
        d = math.pi - d/2
        t1 = t_avg + d
        t2 = t_avg - d
        return (t1, t2)
