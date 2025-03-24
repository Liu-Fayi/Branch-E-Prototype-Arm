import math

class IKSolver():
    def __init__(self, forearm_length, upperarm_length):
        self.r1 = forearm_length
        self.r2 = upperarm_length

    def solve(self, x: float, y: float) -> tuple:
        # Solve the inverse kinematics problem
        # and return the angles of the two joints
        t = math.atan2(y, x)
        u1 = (self.r1**2 + self.r2**2 - x**2 - y**2) / (2 * self.r1 * self.r2)
        u1 = math.acos(d)
        u2 = (self.r1**2 + x**2 + y**2 - self.r2**2) / (2 * self.r1 * math.sqrt(x**2 + y**2))
        u2 = math.acos(u2)
        a1 = t - u2
        a2 = math.pi - u1
        return (a1, a2)
