import board
import busio
import adafruit_vl53l0x
import time
import pwmio
from adafruit_motor import servo
import math
import random

# Configurable parameters
DISTANCE_THRESHOLD = 30  # mm, for edge detection
NUM_READINGS = 5  # Number of readings to average
ANGLE_STEP = 5  # Degrees between measurements
MIN_POINTS = 10  # Minimum points for cylinder fit
FIT_THRESHOLD = 2  # mm, acceptable distance from cylinder surface
angles = list(range(0, 181, ANGLE_STEP))

# Initialize hardware
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    vl53 = adafruit_vl53l0x.VL53L0X(i2c)
    pwm = pwmio.PWMOut(board.GP15, frequency=50, duty_cycle=2 ** 15)
    servo = servo.Servo(pwm)
except Exception as e:
    print(f"Hardware initialization failed: {e}")
    raise

def set_servo_angle(angle):
    angle = max(0, min(180, angle))
    servo.angle = angle
    time.sleep(0.1)

def read_average_distance(num_readings=NUM_READINGS):
    distances = []
    for _ in range(num_readings):
        try:
            distance = vl53.distance  # Distance in mm
            if distance is not None and 10 < distance < 1000:
                distances.append(distance)
            time.sleep(0.05)
        except Exception as e:
            print(f"Distance read error: {e}")
    return sum(distances) / len(distances) if distances else None

def vector_normalize(v):
    length = math.sqrt(sum(x * x for x in v))
    return tuple(x / length if length > 1e-6 else 0 for x in v)

def vector_subtract(v1, v2):
    return tuple(a - b for a, b in zip(v1, v2))

def vector_dot(v1, v2):
    return sum(a * b for a, b in zip(v1, v2))

def distance_point_to_line(point, line_point, line_dir):
    v = vector_subtract(point, line_point)
    t = vector_dot(v, line_dir)
    closest_point = tuple(line_point[i] + t * line_dir[i] for i in range(3))
    diff = vector_subtract(point, closest_point)
    return math.sqrt(vector_dot(diff, diff))

def fit_cylinder(points, max_iterations=1000, sample_size=3):
    """
    Fit a cylinder to 3D points using a RANSAC-like approach.
    Returns axis direction, a point on axis, and radius.
    """
    best_axis = None
    best_point = None
    best_radius = None
    best_inliers = 0

    for _ in range(max_iterations):
        # Randomly sample points
        sample = random.sample(points, sample_size)
        
        # Estimate axis from two points (simplified, assumes third point for validation)
        p1, p2 = sample[0], sample[1]
        axis = vector_normalize(vector_subtract(p2, p1))
        point_on_axis = p1
        
        # Estimate radius by projecting points onto plane perpendicular to axis
        distances = []
        for p in points:
            dist = distance_point_to_line(p, point_on_axis, axis)
            distances.append(dist)
        
        # Median distance as initial radius estimate
        radius = sorted(distances)[len(distances) // 2]
        
        # Count inliers
        inliers = sum(1 for d in distances if abs(d - radius) < FIT_THRESHOLD)
        
        if inliers > best_inliers:
            best_inliers = inliers
            best_axis = axis
            best_point = point_on_axis
            best_radius = radius
    
    if best_inliers < MIN_POINTS:
        return None, None, None
    
    # Refine using all inliers
    inlier_points = [p for p, d in zip(points, distances) if abs(d - best_radius) < FIT_THRESHOLD]
    if len(inlier_points) < MIN_POINTS:
        return None, None, None
    
    # Recalculate axis using PCA-like method on inliers
    centroid = tuple(sum(p[i] for p in inlier_points) / len(inlier_points) for i in range(3))
    xx = xy = xz = yy = yz = zz = 0
    for p in inlier_points:
        dx, dy, dz = vector_subtract(p, centroid)
        xx += dx * dx
        xy += dx * dy
        xz += dx * dz
        yy += dy * dy
        yz += dy * dz
        zz += dz * dz
    
    # Simplified: assume largest variance is along cylinder axis
    det_x = yy * zz - yz * yz
    det_y = xx * zz - xz * xz
    det_z = xx * yy - xy * xy
    if det_z > det_x and det_z > det_y:
        axis = vector_normalize((xy * yz - xz * yy, xz * yz - xy * zz, det_z))
    elif det_y > det_x:
        axis = vector_normalize((xz * yz - xy * zz, det_y, xy * xz - yz * xx))
    else:
        axis = vector_normalize((det_x, xy * xz - yz * xx, xz * xy - yz * det_x))
    
    # Refine radius
    distances = [distance_point_to_line(p, centroid, axis) for p in inlier_points]
    radius = sum(distances) / len(distances)
    
    return axis, centroid, radius

# Scanning process

data = []

for angle in angles:
    set_servo_angle(angle)
    distance = read_average_distance()
    if distance is not None:
        # Convert to 3D coordinates (assuming sensor at origin, z along distance)
        angle_rad = math.radians(angle)
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        z = 0  
        data.append((x, y, z, distance))

if len(data) < MIN_POINTS:
    print("Error: Insufficient valid distance data collected")
else:
    points = [(x, y, z) for x, y, z, _ in data]
    distances = [d for _, _, _, d in data]
    angles_measured = [a for a in angles[:len(distances)]]
    
    # Detect potential cylinder region
    gradients = [0]  # Initial gradient
    for i in range(1, len(distances)):
        angle_diff = angles_measured[i] - angles_measured[i-1]
        dist_diff = distances[i] - distances[i-1]
        gradients.append(dist_diff / angle_diff if angle_diff != 0 else 0)
    
    # Find region with consistent measurements
    cylinder_points = []
    start_idx = None
    for i in range(1, len(distances) - 1):
        if (abs(distances[i] - distances[i-1]) < DISTANCE_THRESHOLD and 
            abs(gradients[i]) < 10):  # Low gradient for cylinder surface
            if start_idx is None:
                start_idx = i - 1
            cylinder_points.append(points[i])
        elif start_idx is not None:
            break
    
    if len(cylinder_points) < MIN_POINTS:
        print("Failed to detect sufficient cylinder points")
    else:
        # Fit cylinder
        axis, center, radius = fit_cylinder(cylinder_points)
        
        if axis is None:
            print("Failed to fit cylinder to data")
        else:
            # Calculate center angle
            center_angle = (angles_measured[start_idx] + angles_measured[start_idx + len(cylinder_points) - 1]) / 2
            avg_distance = sum(distances[start_idx:start_idx + len(cylinder_points)]) / len(cylinder_points)
            
            # Calculate fit quality
            residuals = [distance_point_to_line(p, center, axis) - radius for p in cylinder_points]
            fit_error = sum(abs(r) for r in residuals) / len(residuals)
            
            print(f"Cylinder detected:")
            print(f"Center angle: {center_angle:.1f} degrees")
            print(f"Average distance: {avg_distance:.1f} mm")
            print(f"Radius: {radius:.1f} mm")
            print(f"Center position: (x={center[0]:.1f}, y={center[1]:.1f}, z={center[2]:.1f}) mm")
            print(f"Axis direction: ({axis[0]:.3f}, {axis[1]:.3f}, {axis[2]:.3f})")
            print(f"Fit error: {fit_error:.1f} mm")
            if fit_error > FIT_THRESHOLD:
                print("Warning: Cylinder fit quality may be poor")