import math

class IKSolver():
    def __init__(self, forearm_length, upperarm_length):
        # Initialize the inverse kinematics solver with link lengths of the arm
        self.r2 = forearm_length  # Forearm length (second arm segment)
        self.r1 = upperarm_length  # Upper arm length (first arm segment)

    def solve(self, x: float, y: float) -> tuple:
        # Solve the inverse kinematics problem for a 2-link arm
        # Returns the joint angles needed to reach the point (x, y)
        
        t = math.atan2(y, x)
        # print(t)
        u1 = (self.r1**2 + self.r2**2 - x**2 - y**2) / (2 * self.r1 * self.r2)
        u1 = math.acos(u1)
        u2 = (self.r1**2 + x**2 + y**2 - self.r2**2) / (2 * self.r1 * math.sqrt(x**2 + y**2))
        # print(u2)
        u2 = math.acos(u2)
        # print(u1, u2)
        a1 = t - u2
        a2 = math.pi - u1
        return (a1, a2)
