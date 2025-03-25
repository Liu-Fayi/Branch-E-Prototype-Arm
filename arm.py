from ik import IKSolver
from servo import Servo
from stepper import Stepper
import math
import board

class Arm:
    def __init__(self):
        self.forearm_length = 12.647
        self.upperarm_length = 16.5
        self.ik_solver = IKSolver(self.forearm_length, self.upperarm_length)
        self.base_rotate = Stepper(board.GP0, board.GP1, board.GP2, board.GP3)
        self.z_movement = Stepper(board.GP4, board.GP5, board.GP6, board.GP7)
        self.elbow = Servo(board.GP8, 50, 2 ** 15)
        self.wrist = Servo(board.GP9, 50, 2 ** 15)
        self.claw = Servo(board.GP10, 50, 2 ** 15)

    def move_to(self, x, y, z, wrist_angle):
        t1, t2 = self.ik_solver.solve(x, y)
        self.base_rotate_to(t1)
        self.z_move_to(z)
        self.elbow_move(t2)
        self.wrist_move(wrist_angle)

    def open_claw(self):
        self.claw.set_angle(0)
    
    def close_claw(self):
        self.claw.set_angle(70)

    def base_rotate(self, angle):
        if angle > 0:
            self.base_rotate.turn_to(int(math.degrees(angle)*10/1.8), True)
        else:
            self.base_rotate.turn_to(int(math.degrees(angle)*10/1.8), False)

    def base_rotate_to(self, angle):
        self.base_rotate.turn_to_target(int(math.degrees(angle)*10/1.8))

    def z_move(self, distance):
        if distance > 0:
            self.z_movement.turn_to(int(distance*10/8*360/1.8), True)
        else:
            self.z_movement.turn_to(int(distance*10/8*360/1.8), False)

    def z_move_to(self, distance):
        self.z_movement.turn_to_target(int(distance*10/8*360/1.8))

    def elbow_move(self, angle):
        self.elbow.set_angle(math.degrees(angle))

    def wrist_move(self, angle):
        self.wrist.set_angle(math.degrees(angle))
