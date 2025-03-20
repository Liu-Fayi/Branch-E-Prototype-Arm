from ik import IKSolver
from servo import Servo
from stepper import Stepper
import board

class Arm:
    def __init__(self):
        self.forearm_length = 0.1
        self.upperarm_length = 0.1
        self.ik_solver = IKSolver(self.forearm_length, self.upperarm_length)
        self.base_rotate = Stepper(board.GP0, board.GP1, board.GP2, board.GP3)
        self.z_movement = Stepper(board.GP4, board.GP5, board.GP6, board.GP7)
        self.elbow = Servo(board.GP8, 50, 2 ** 15)
        self.wrist = Servo(board.GP9, 50, 2 ** 15)
        self.claw = Servo(board.GP10, 50, 2 ** 15)

    def move_to(self, x, y, z, wrist_angle):
        t1, t2 = self.ik_solver.solve(x, y)
        