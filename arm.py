from ik import IKSolver
from servo import Servo
from stepper import Stepper
import math
import board
import busio
from digitalio import DigitalInOut
import struct

X_OFF = 0
Y_OFF = 0



i2c = busio.I2C(board.GP17, board.GP16)
SLAVE_ADDRESS = 0x08
i2c.configure(address = SLAVE_ADDRESS, slave = True)




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


def send_go_signal(i2c):
    while not i2c.try_lock():
        pass
    try:
        i2c.writeto(SLAVE_ADDRESS, bytes([0x01]))
    finally:
        i2c.unlock() 


def main():
    arm = Arm()
    send_go_signal(i2c)
    while True:
        if i2c.requested():
            buffer = i2c.read(24)
            if buffer and len(buffer) == 24:
                x, y, z, orient_x, orient_y, _ = struct.unpack('ffffff', buffer)
                arm.move_to(x + X_OFF , y + Y_OFF , 10 , math.atan2(orient_y, orient_x))
                arm.open_claw()
                arm.move_to(x + X_OFF , y + Y_OFF , 3 , math.atan2(orient_y, orient_x))
                arm.close_claw()
                arm.move_to(x + X_OFF , y + Y_OFF , 10 , math.atan2(orient_y, orient_x))
                arm.move_to(0, 0, 10, 0)
                break
            else:
                print("Invalid data received")


if __name__ == "__main__":
    main()
                
    
