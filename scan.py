import board
import busio
import adafruit_vl53l0x
import time
import pwmio
from adafruit_motor import servo
import math

THRESHOLD = 50 



# Initialize I2C and VL53L0X ToF sensor
i2c = busio.I2C(board.SCL, board.SDA)
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

# Initialize PWM and servo on pin GP15
pwm = pwmio.PWMOut(board.GP15, frequency=50, duty_cycle=2 ** 15)
servo = servo.Servo(pwm)


def set_servo_angle(angle):
    servo.angle = angle
    time.sleep(0.1)  # Allow time for servo to reach position

# Read average distance from ToF sensor
def read_average_distance(num_readings=5):
    distances = []
    for _ in range(num_readings):
        distance = vl53.distance  # Distance in millimeters
        if distance is not None:  # Check for valid reading
            distances.append(distance)
        time.sleep(0.05)  # Small delay between readings
    return sum(distances) / len(distances) if distances else None

def fit_plane(points):
    # Simple plane fitting without NumPy
    # Calculate centroid
    n = len(points)
    sum_x = sum(p[0] for p in points)
    sum_y = sum(p[1] for p in points)
    sum_z = sum(p[2] for p in points)
    centroid = (sum_x/n, sum_y/n, sum_z/n)
    
    # Calculate covariance matrix components
    xx = sum((p[0] - centroid[0])**2 for p in points)
    xy = sum((p[0] - centroid[0])*(p[1] - centroid[1]) for p in points)
    xz = sum((p[0] - centroid[0])*(p[2] - centroid[2]) for p in points)
    yy = sum((p[1] - centroid[1])**2 for p in points)
    yz = sum((p[1] - centroid[1])*(p[2] - centroid[2]) for p in points)
    
    # Find normal vector 
    # For a best-fit plane, normal vector is proportional to:
    a = xy*yz - yy*xz
    b = xz*xy - xx*yz
    c = xx*yy - xy*xy
    
    # Normalize
    norm_length = math.sqrt(a**2 + b**2 + c**2)
    if norm_length < 1e-6:  
        normal = (0, 0, 1)  # Default to upward normal
    else:
        normal = (a/norm_length, b/norm_length, c/norm_length)
    
    # Calculate d in equation: ax + by + cz + d = 0
    d = -normal[0]*centroid[0] - normal[1]*centroid[1] - normal[2]*centroid[2]
    
    return normal, d


# Define angles to scan (0 to 180 degrees, 5-degree steps)
angles = list(range(0, 181, 5))

data = []
for angle in angles:
    set_servo_angle(angle)
    distance = read_average_distance()
    if distance is not None:
        data.append((angle, distance))
    else:
        print(f"Warning: No valid distance at angle {angle}")


if not data:
    print("Error: No valid distance data collected")
else:
    
    distances = [d for _, d in data]

    # Define threshold for detecting edges (in mm)
    threshold = THRESHOLD  # Adjust based on environment and rectangle size

    # Find left edge (significant decrease in distance)
    left_index = None
    for i in range(len(angles) - 1):
        diff = distances[i + 1] - distances[i]
        if diff < -threshold:
            left_index = i
            break

    if left_index is None:
        print("Left edge of rectangle not found")
    else:
        # Find right edge (significant increase in distance)
        right_index = None
        for j in range(left_index + 1, len(angles) - 1):
            diff = distances[j + 1] - distances[j]
            if diff > threshold:
                right_index = j
                break

        if right_index is None:
            print("Right edge of rectangle not found")
        else:
            # Calculate center angle and average distance
            center_angle = (angles[left_index] + angles[right_index]) / 2
            rectangle_distance = sum(distances[left_index:right_index + 1]) / (right_index - left_index + 1)
            fit_plane_points = []
            for i in range(left_index, right_index + 1):
                angle = angles[i]
                distance = distances[i]
                x = distance * math.cos(math.radians(angle))
                y = distance * math.sin(math.radians(angle))
                fit_plane_points.append((x, y, distance))
            normal, d = fit_plane(fit_plane_points)

            
            print(f"Rectangle localized:")
            print(f"Center angle: {center_angle} degrees")
            print(f"Distance: {rectangle_distance} mm")
            print(f"Plane normal: {normal}")
            print(f"Plane offset: {d}")