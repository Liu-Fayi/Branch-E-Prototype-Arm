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
    def __init__(self, sensor_x=-105, sensor_y=96, sensor_angle=15):
        self.zero_base_angle = math.radians(268)
        self.wrist_offset = math.radians(43)
        self.sensor_x = sensor_x
        self.sensor_y = sensor_y
        self.sensor_angle = math.radians(sensor_angle)
        self.forearm_length = 126.47
        self.upperarm_length = 165
        self.ik_solver = IKSolver(self.forearm_length, self.upperarm_length)
        self.base_rotation = Stepper(board.GP4, board.GP5, board.GP15)
        self.z_movement = Stepper(board.GP2, board.GP3, board.GP14)
        self.elbow = Servo(board.GP8, 50, 2 ** 15)
        self.wrist = Servo(board.GP9, 50, 2 ** 15)
        self.claw = Servo(board.GP7, 50, 2 ** 15)
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
            print(x, y, z, math.degrees(wrist_angle))
        t1, t2 = self.ik_solver.solve(x, y)
        if math.isnan(t1) or math.isnan(t2):
            print("No solution found, going to max r")
            t1 = math.atan2(y, x)
            t2 = 0
        print(math.degrees(t1), math.degrees(t2))
        self.base_rotation.set_velocity(150)
        self.z_movement.set_velocity(800)
        self.elbow_move(t2)
        wrist_angle = - t1 - t2 + wrist_angle + math.pi/2
        print(math.degrees(wrist_angle))
        self.wrist_move(wrist_angle)
        self.base_rotate_to(t1)
        self.z_move_to(z)

    def open_claw(self):
        self.claw.set_angle(0)

    def close_claw(self):
        self.claw.set_angle(52)

    def base_rotate(self, angle):
        self.base_rotation.set_velocity(150)
        if angle > 0:
            self.base_rotation.turn_to(int(math.degrees(angle)*100/24/1.8*2), True)
        else:
            print(int(-math.degrees(angle)*10/1.8*2))
            self.base_rotation.turn_to(int(-math.degrees(angle)*100/24/1.8*2), False)

    def base_rotate_to(self, angle):
        self.base_rotation.set_velocity(150)
        self.base_rotation.turn_to_target(int(math.degrees(angle-self.zero_base_angle)*100/24/1.8*2))

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
        angle = -angle
        while angle < math.radians(-33.4):
            angle += math.pi
        while angle > math.radians(106.5):
            angle -= math.pi
        if angle < math.radians(-33.4): 
            if abs(angle - math.radians(-33.4)) < abs(angle - math.radians(106.5) + math.pi):
                angle = math.radians(-33.4)
            else:
                angle = math.radians(106.5)
        elif angle > math.radians(106.5):
            if abs(angle - math.radians(106.5)) < abs(angle - math.radians(-33.4) - math.pi):
                angle = math.radians(106.5)
            else:
                angle = math.radians(-33.4)
        self.wrist.set_angle(math.degrees(9/7*angle + self.wrist_offset))

    def calibrate_base(self):
        step_size = 1
        while self.base_limit.value:
            self.base_rotation.turn_vel(True, 200)
        while not self.base_limit.value:
            self.base_rotation.turn_vel(False, 20)
        self.base_rotation.reset_position()

    def calibrate_z(self):
        step_size = 1
        while self.z_limit.value:
            # print("down")
            self.z_movement.turn_vel(False, 1000)
        while not self.z_limit.value:
            self.z_movement.turn_vel(True, 100)
        self.z_movement.reset_position()

    def calibrate(self):
        self.open_claw()
        self.elbow_move(0)
        self.calibrate_z()
        time.sleep(0.5)
        self.z_move_to(25)
        self.calibrate_base()

    def sleep_steppers(self):
        self.base_rotation.sleep()
        self.z_movement.sleep()

    def collect_branch(self):
        self.z_move_to(30)
        self.elbow_move(0)
        self.wrist_move(0)
        self.base_rotate_to(math.radians(260))
        self.open_claw()
        time.sleep(1)

