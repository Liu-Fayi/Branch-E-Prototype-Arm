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

# Define pins for the base and Z-axis limit switches
BASE_LIMIT_PIN = board.GP0
Z_LIMIT_PIN = board.GP1

class Arm:
    def __init__(self, sensor_x=-108, sensor_y=100, sensor_angle=15):
        # Initialize basic configuration parameters converting degrees to radians when needed
        self.zero_base_angle = math.radians(268)     # Zero reference for base rotation
        self.wrist_offset = math.radians(43)          # Offset for wrist positioning
        self.sensor_x = sensor_x                      # X offset of sensor relative to arm base
        self.sensor_y = sensor_y                      # Y offset of sensor relative to arm base
        self.sensor_angle = math.radians(sensor_angle)  # Sensor mounting angle (in radians)
        
        # Arm lengths in mm (or chosen units) used for inverse kinematics
        self.forearm_length = 126.47
        self.upperarm_length = 165
        # Initialize the kinematics solver instance using link lengths
        self.ik_solver = IKSolver(self.forearm_length, self.upperarm_length)
        
        # Set up motor controllers:
        # Base rotation and vertical (Z-axis) movement use stepper motors.
        self.base_rotation = Stepper(board.GP4, board.GP5, board.GP15)
        self.z_movement = Stepper(board.GP2, board.GP3, board.GP14)
        
        # The arm’s elbow, wrist, and claw are controlled by servos.
        self.elbow = Servo(board.GP8, 50, 2 ** 15)
        self.wrist = Servo(board.GP9, 50, 2 ** 15)
        self.claw = Servo(board.GP7, 50, 2 ** 15)
        
        # Initialize limit switches for base and vertical movements
        self.base_limit = digitalio.DigitalInOut(BASE_LIMIT_PIN)
        self.z_limit = digitalio.DigitalInOut(Z_LIMIT_PIN)
        for limit in (self.base_limit, self.z_limit):
            limit.direction = digitalio.Direction.INPUT
            limit.pull = digitalio.Pull.DOWN

    def transform_sensor_to_arm(self, x, y, angle):
        # Convert sensor readings into arm coordinates.
        # 'angle' should be given in degrees.
        x_arm = self.sensor_x + x * math.cos(self.sensor_angle) - y * math.sin(self.sensor_angle)
        y_arm = self.sensor_y + x * math.sin(self.sensor_angle) + y * math.cos(self.sensor_angle)
        # Adjust wrist angle by adding sensor's mounting angle
        angle_arm = angle + self.sensor_angle
        return x_arm, y_arm, angle_arm

    def move_to(self, x, y, z, wrist_angle, from_sensor=True):
        # Move the arm to a target (x, y, z) position with the specified wrist angle.
        # If from_sensor is True, convert sensor coordinates to arm coordinates first.
        if from_sensor:
            x, y, wrist_angle = self.transform_sensor_to_arm(x, y, wrist_angle)
            print(x, y, z, math.degrees(wrist_angle))
        # Use inverse kinematics to obtain joint angles for target (x, y)
        t1, t2 = self.ik_solver.solve(x, y)
        if math.isnan(t1) or math.isnan(t2):
            print("No solution found, going to max r")
            t1 = math.atan2(y, x)
            t2 = 0
        print(math.degrees(t1), math.degrees(t2))
        # Set velocities for smooth movement on base and z-axis
        self.base_rotation.set_velocity(750)
        self.z_movement.set_velocity(1500)
        # Move the elbow to the angle t2 (forearm joint)
        self.elbow_move(t2)
        # Adjust wrist: combine computed joint angles with desired wrist angle and a 90° offset to match physical setup.
        wrist_angle = - t1 - t2 + wrist_angle + math.pi/2
        print(math.degrees(wrist_angle))
        self.wrist_move(wrist_angle)
        # Rotate the base according to t1.
        self.base_rotate_to(t1)
        # Move vertical axis to the target height z.
        self.z_move_to(z)

    def open_claw(self):
        # Opens the claw by setting its angle to 0°
        self.claw.set_angle(0)

    def close_claw(self):
        # Closes the claw by setting its angle to 52° (empirically chosen)
        self.claw.set_angle(52)

    def base_rotate(self, angle):
        # Rotate the base by a relative angle.
        self.base_rotation.set_velocity(150)
        if angle > 0:
            self.base_rotation.turn_to(int(math.degrees(angle)*100/24/1.8*2), True)
        else:
            print(int(-math.degrees(angle)*10/1.8*2))
            self.base_rotation.turn_to(int(-math.degrees(angle)*100/24/1.8*2), False)

    def base_rotate_to(self, angle):
        # Rotate the base to an absolute target angle given as radians.
        self.base_rotation.set_velocity(150)
        # The calculation adjusts the angle with a zero reference.
        self.base_rotation.turn_to_target(int(math.degrees(angle - self.zero_base_angle)*100/24/1.8*2))

    def z_move(self, distance):
        # Move vertically a relative distance.
        if distance > 0:
            self.z_movement.turn_to(int(distance*10/8*360/1.8*2), True)
        else:
            self.z_movement.turn_to(int(-distance*10/8*360/1.8*2), False)

    def z_move_to(self, distance):
        # Move vertically to an absolute position.
        self.z_movement.turn_to_target(int(distance*10/8*360/1.8))

    def elbow_move(self, angle):
        # Move the elbow servo to the specified angle (in radians)
        self.elbow.set_angle(math.degrees(angle))

    def wrist_move(self, angle):
        # Adjust and move the wrist servo to the given angle (in radians)
        # Negate angle (physical rotation is clockwise positive) then normalize within valid range [-33.4°, 106.5°], this is determined emperically, the wrist servo did not have a 180 degrees range.
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
        # Apply a scaling factor (9/7, determined emperically, the servo did not function as advertized) and add an offset; these values are tuned for proper positioning.
        self.wrist.set_angle(math.degrees(9/7 * angle + self.wrist_offset))

    def calibrate_base(self):
        # Calibrate the base rotation using a limit switch.
        # First, gently move until the limit is reached; then back off until the limit is released.
        step_size = 1
        while self.base_limit.value:
            self.base_rotation.turn_vel(True, 200)
        while not self.base_limit.value:
            self.base_rotation.turn_vel(False, 20)
        # Reset the step counter after calibration.
        self.base_rotation.reset_position()

    def calibrate_z(self):
        # Calibrate the vertical (Z) movement using the corresponding limit switch.
        step_size = 1
        while self.z_limit.value:
            # print("down")
            self.z_movement.turn_vel(False, 1000)
        while not self.z_limit.value:
            self.z_movement.turn_vel(True, 100)
        self.z_movement.reset_position()

    def calibrate(self):
        # Overall arm calibration routine:
        # 1. Open claw and set elbow to zero.
        # 2. Calibrate the vertical axis.
        # 3. Move up to avoid wiring and calibrate the base rotation.
        self.open_claw()
        self.elbow_move(0)
        self.calibrate_z()
        time.sleep(0.5)
        self.z_move_to(25)
        self.calibrate_base()

    def sleep_steppers(self):
        # Put stepper motors into sleep mode to release torque (allow free spinning)
        self.base_rotation.sleep()
        self.z_movement.sleep()

    def collect_branch(self):
        # High-level routine for branch collection into bin at the back:
        # 1. Move vertical axis to a preset value to avoid wiring.
        # 2. Align elbow and wrist for drop.
        # 3. Rotate base to the back.
        # 4. Open claw to release.
        self.base_rotation.set_velocity(200)
        self.z_movement.set_velocity(800)
        self.z_move_to(30)
        self.elbow_move(0)
        self.wrist_move(0)
        self.base_rotate_to(math.radians(260))
        self.open_claw()
        time.sleep(1)

