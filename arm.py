from ik import IKSolver
from servo import Servo
from stepper import Stepper
import math
import board
import busio
from digitalio import DigitalInOut
import struct
import digitalio
import time

BASE_LIMIT_PIN = board.GP0
Z_LIMIT_PIN = board.GP1


class Arm:
    def __init__(self, sensor_x=0, sensor_y=0, sensor_angle=0):
        self.sensor_x = sensor_x
        self.sensor_y = sensor_y
        self.sensor_angle = sensor_angle
        self.forearm_length = 12.647
        self.upperarm_length = 16.5
        self.ik_solver = IKSolver(self.forearm_length, self.upperarm_length)
        self.base_rotate = Stepper(board.GP4, board.GP5)
        self.z_movement = Stepper(board.GP2, board.GP3)
        self.elbow = Servo(board.GP8, 50, 2 ** 15)
        self.wrist = Servo(board.GP9, 50, 2 ** 15)
        self.claw = Servo(board.GP10, 50, 2 ** 15)
        self.base_limit = digitalio.DigitalInOut(BASE_LIMIT_PIN)
        self.z_limit = digitalio.DigitalInOut(Z_LIMIT_PIN)
        for limit in (self.base_limit, self.z_limit):
            limit.direction = digitalio.Direction.INPUT
            limit.pull = digitalio.Pull.DOWN

    def transform_sensor_to_arm(self, x, y, angle):
        theta = math.radians(angle)
        x_arm = self.sensor_x + x * math.cos(theta) - y * math.sin(theta)
        y_arm = self.sensor_y + x * math.sin(theta) + y * math.cos(theta)
        angle_arm = angle + self.sensor_angle
        return x_arm, y_arm, angle_arm

    def move_to(self, x, y, z, wrist_angle, from_sensor=True):
        if from_sensor:
            x, y, wrist_angle = self.transform_sensor_to_arm(x, y, wrist_angle)
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
            self.base_rotate.turn_to(int(math.degrees(angle)*10/1.8*2), True)
        else:
            self.base_rotate.turn_to(int(-math.degrees(angle)*10/1.8*2), False)

    def base_rotate_to(self, angle):
        self.base_rotate.turn_to_target(int(math.degrees(angle)*10/1.8*2))

    def z_move(self, distance):
        if distance > 0:
            self.z_movement.turn_to(int(distance*10/8*360/1.8*2), True)
        else:
            self.z_movement.turn_to(int(-distance*10/8*360/1.8*2), False)

    def z_move_to(self, distance):
        self.z_movement.turn_to_target(int(distance*10/8*360/1.8))

    def elbow_move(self, angle):
        self.elbow.set_angle(math.degrees(angle))

    def wrist_move(self, angle):
        self.wrist.set_angle(math.degrees(angle))

    def calibrate_base(self):
        step_size = 1
        while self.base_limit.value:
            self.base_rotate.DELAY = 0.005
            self.base_rotate.turn(True)
        while not self.base_limit.value:
            self.base_rotate.DELAY = 0.1
            self.base_rotate.turn(False)
        self.base_rotate.reset_position()

    def calibrate_z(self):
        step_size = 1
        while self.z_limit.value:
            self.z_movement.DELAY = 0.001
            self.z_movement.turn(False)
        while not self.base_limit.value:
            self.z_movement.DELAY = 0.01
            self.z_movement.turn(True)
        self.z_movement.reset_position()

