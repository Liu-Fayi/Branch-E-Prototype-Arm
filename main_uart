from arm import Arm
from uart import UARTRecever
import time
import math

X_OFF = 0
Y_OFF = 0
ANGLE_OFF = 0


def transform_sensor_to_arm(x, y, z, angle):
    theta = math.radians(angle)
    x_arm = X_OFF + x * math.cos(theta) - y * math.sin(theta)
    y_arm = Y_OFF + x * math.sin(theta) + y * math.cos(theta)
    angle_arm = angle + ANGLE_OFF
    return x_arm, y_arm, z, angle_arm


def main():
    arm = Arm()
    uart = UARTRecever()
    uart.send_go_signal()
    arm.calibrate_base()
    arm.calibrate_z()

    x,y,_,angle = uart.wait_for_data()

    ax, ay ,_, a_angle = transform_sensor_to_arm(x, y, 0, angle)
    arm.move_to(ax, ay, 10, a_angle)
    arm.open_claw()
    time.sleep(1)
    arm.move_to(ax, ay, 3, a_angle)
    arm.close_claw()
    time.sleep(1)
    arm.move_to(ax, ay, 10, a_angle)
    arm.move_to(0, 0, 10, 0)
    arm.open_claw()

if __name__ == "__main__":
    main()

    

