import ik
import math

class Transform:
    def __init__(self, sensor_x, sensor_y, sensor_angle):
        self.sensor_x = sensor_x
        self.sensor_y = sensor_y
        self.sensor_angle = sensor_angle

    def transform_sensor_to_arm(self, x, y, angle):
        theta = math.radians(angle)
        x_arm = self.sensor_x + x * math.cos(theta) - y * math.sin(theta)
        y_arm = self.sensor_y + x * math.sin(theta) + y * math.cos(theta)
        angle_arm = angle + self.sensor_angle
        return x_arm, y_arm, angle_arm

i = ik.IKSolver(12.647, 16.5)
tr = Transform(-105, 85, 15)
res = tr.transform_sensor_to_arm(67, 121, 3)
print(i.solve(0, 0))