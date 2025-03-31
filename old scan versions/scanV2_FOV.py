# Save as main.py on CIRCUITPY drive
import board
import busio
import adafruit_vl53l1x
import time
import pwmio
import adafruit_motor.servo
import digitalio

# Constants
ANGLE_RANGE = (0, 100)
ANGLE_STEP = 1
NUM_READINGS = 20
SMOOTH_STEP = 1
SMOOTH_DELAY = 0.02
INTERRUPT_TIMEOUT = .1
MIN_EDGE_POINTS = 4

# Trimmed trig tables (0° to 90°, 1° steps, 91 values)
COS_TABLE = (1.0000, 0.9998, 0.9994, 0.9986, 0.9976, 0.9962, 0.9945, 0.9925, 0.9903, 0.9877,
             0.9848, 0.9816, 0.9781, 0.9744, 0.9703, 0.9659, 0.9613, 0.9563, 0.9511, 0.9455,
             0.9397, 0.9336, 0.9272, 0.9205, 0.9135, 0.9063, 0.8988, 0.8910, 0.8829, 0.8746,
             0.8660, 0.8572, 0.8480, 0.8387, 0.8290, 0.8192, 0.8090, 0.7986, 0.7880, 0.7771,
             0.7660, 0.7547, 0.7431, 0.7314, 0.7193, 0.7071, 0.6947, 0.6820, 0.6691, 0.6561,
             0.6428, 0.6293, 0.6157, 0.6018, 0.5878, 0.5736, 0.5592, 0.5446, 0.5299, 0.5150,
             0.5000, 0.4848, 0.4695, 0.4540, 0.4384, 0.4226, 0.4067, 0.3907, 0.3746, 0.3584,
             0.3420, 0.3256, 0.3090, 0.2924, 0.2756, 0.2588, 0.2419, 0.2250, 0.2079, 0.1908,
             0.1736, 0.1564, 0.1392, 0.1219, 0.1045, 0.0872, 0.0698, 0.0523, 0.0349, 0.0175,
             0.0000)
SIN_TABLE = (0.0000, 0.0175, 0.0349, 0.0523, 0.0698, 0.0872, 0.1045, 0.1219, 0.1392, 0.1564,
             0.1736, 0.1908, 0.2079, 0.2250, 0.2419, 0.2588, 0.2756, 0.2924, 0.3090, 0.3256,
             0.3420, 0.3584, 0.3746, 0.3907, 0.4067, 0.4226, 0.4384, 0.4540, 0.4695, 0.4848,
             0.5000, 0.5150, 0.5299, 0.5446, 0.5592, 0.5736, 0.5878, 0.6018, 0.6157, 0.6293,
             0.6428, 0.6561, 0.6691, 0.6820, 0.6947, 0.7071, 0.7193, 0.7314, 0.7431, 0.7547,
             0.7660, 0.7771, 0.7880, 0.7986, 0.8090, 0.8192, 0.8290, 0.8387, 0.8480, 0.8572,
             0.8660, 0.8746, 0.8829, 0.8910, 0.8988, 0.9063, 0.9135, 0.9205, 0.9272, 0.9336,
             0.9397, 0.9455, 0.9511, 0.9563, 0.9613, 0.9659, 0.9703, 0.9744, 0.9781, 0.9816,
             0.9848, 0.9877, 0.9903, 0.9925, 0.9945, 0.9962, 0.9976, 0.9986, 0.9994, 0.9998,
             1.0000)

def initialize_hardware(interrupt_pin=board.GP18):
    print("Initializing hardware...")
    i2c = busio.I2C(scl=board.GP17, sda=board.GP16)
    interrupt = digitalio.DigitalInOut(interrupt_pin)
    interrupt.direction = digitalio.Direction.INPUT
    interrupt.pull = digitalio.Pull.UP
    vl53 = adafruit_vl53l1x.VL53L1X(i2c)
    vl53.distance_mode = 1

    #print i2c device address
    print(i2c.scan())
    i2c.writeto(vl53._i2c_addr, bytes([0x7F, 0x07]))  # Select ROI register bank
    i2c.writeto(vl53._i2c_addr, bytes([0x09, 0x00, 0x06, 0x02]))  
    i2c.writeto(vl53._i2c_addr, bytes([0x0A, 0x00, 0x09, 0x07]))  
    vl53.start_ranging()
    pwm = pwmio.PWMOut(board.GP1, frequency=50, duty_cycle=2**15)
    servo = adafruit_motor.servo.Servo(pwm)
    if not wait_for_interrupt(vl53, interrupt):
        raise RuntimeError("VL53L1X not responding")
    distance = vl53.distance
    if distance is None:
        raise RuntimeError("VL53L1X initial reading failed")
    print(f"Initial distance reading: {distance} cm")
    return vl53, servo, 0, interrupt

def wait_for_interrupt(vl53, interrupt, timeout=INTERRUPT_TIMEOUT):
    start_time = time.monotonic()
    vl53.clear_interrupt()
    while time.monotonic() - start_time < timeout:
        if not interrupt.value:
            return True
        time.sleep(0.001)
    print("Interrupt timeout")
    return False

def set_servo_angle(servo, current_angle, target_angle, step=SMOOTH_STEP, delay=SMOOTH_DELAY):
    target_angle = max(0, min(180, target_angle))
    if current_angle is None:
        current_angle = target_angle
    steps = int(abs(target_angle - current_angle) / step) + 1
    for i in range(steps):
        interp_angle = current_angle + (target_angle - current_angle) * i / steps
        servo.angle = interp_angle
        time.sleep(delay)
    servo.angle = target_angle
    return target_angle

def read_average_distance(vl53, interrupt, num_readings=NUM_READINGS):
    total = 0
    count = 0
    for _ in range(num_readings):
        if wait_for_interrupt(vl53, interrupt):
            distance = vl53.distance
            if distance is not None:
                distance *= 10  # cm to mm
                if 10 < distance < 4000:
                    total += distance
                    count += 1
            vl53.clear_interrupt()
    return total / count if count > 0 else None

def scan(vl53, servo, interrupt, current_angle):
    angles = [ANGLE_RANGE[0] + i * ANGLE_STEP for i in range(int((ANGLE_RANGE[1] - ANGLE_RANGE[0]) / ANGLE_STEP) + 1)]
    data = []
    print("Starting scan...")
    for angle in angles:
        current_angle = set_servo_angle(servo, current_angle, angle)
        distance = read_average_distance(vl53, interrupt)
        data.append((angle, distance))
        print(f"Angle: {angle}, Distance: {distance if distance is not None else 'None'} mm")
    return data, current_angle

def detect_edges(data):
    valid_data = [(a, d) for a, d in data if d is not None]
    if len(valid_data) < MIN_EDGE_POINTS:
        print(f"Error: Insufficient valid data points: {len(valid_data)}")
        return [], valid_data

    angles, distances = zip(*valid_data)
    slopes = [0]
    for i in range(1, len(distances)):
        slopes.append((distances[i] - distances[i-1]) / ANGLE_STEP)

    smoothed = [slopes[0]]
    for i in range(1, len(slopes) - 1):
        smoothed.append((slopes[i-1] + slopes[i] + slopes[i+1]) / 3)
    smoothed.append(slopes[-1])

    for i in range(len(angles)):
        print(f"Angle: {angles[i]}, Slope: {smoothed[i]:.2f}")

    min_slope = float('inf')
    max_slope = float('-inf')
    min_idx = max_idx = None

    for i in range(1, len(smoothed) - 1):
        if smoothed[i] < min_slope:
            min_slope = smoothed[i]
            min_idx = i
        if smoothed[i] > max_slope:
            max_slope = smoothed[i]
            max_idx = i

    if min_idx is not None and max_idx is not None and min_idx < max_idx:
        edges = [
            (angles[min_idx-1], distances[min_idx-1], angles[min_idx], distances[min_idx]),
            (angles[max_idx-1], distances[max_idx-1], angles[max_idx], distances[max_idx])
        ]
        print(f"Negative edge: Angle {angles[min_idx-1]} to {angles[min_idx]}, Distance {distances[min_idx-1]:.1f} to {distances[min_idx]:.1f}, Slope {min_slope:.2f}")
        print(f"Positive edge: Angle {angles[max_idx-1]} to {angles[max_idx]}, Distance {distances[max_idx-1]:.1f} to {distances[max_idx]:.1f}, Slope {max_slope:.2f}")
    else:
        print("No valid edges detected")
        edges = []

    return edges, valid_data

def calculate_object_properties(edges, data):
    if not edges or len(edges) < 2:
        print("No valid object edges detected")
        return None, None, None, None

    valid_data = [(a, d) for a, d in data if d is not None]
    angles, distances = zip(*valid_data)
    start_idx = [i for i, (a, _) in enumerate(valid_data) if a == edges[0][2]][0]
    end_idx = [i for i, (a, _) in enumerate(valid_data) if a == edges[1][0]][0]

    # Centroid
    sum_angle_dist = sum_dist = 0
    for i in range(start_idx, end_idx + 1):
        angle, dist = valid_data[i]
        sum_angle_dist += angle * dist
        sum_dist += dist
    center_angle = sum_angle_dist / sum_dist if sum_dist > 0 else (edges[0][2] + edges[1][0]) / 2
    center_dist = sum(distances[start_idx:end_idx+1]) / (end_idx - start_idx + 1)

    angle_idx = int(center_angle)
    center_x = center_dist * COS_TABLE[angle_idx]
    center_y = center_dist * SIN_TABLE[angle_idx]
    center_z = 0

    # Least-squares fit
    n = end_idx - start_idx + 1
    sum_x = sum_y = sum_xy = sum_xx = 0
    for i in range(start_idx, end_idx + 1):
        angle = int(valid_data[i][0])
        x = valid_data[i][1] * COS_TABLE[angle]
        y = valid_data[i][1] * SIN_TABLE[angle]
        sum_x += x
        sum_y += y
        sum_xy += x * y
        sum_xx += x * x

    denom = n * sum_xx - sum_x * sum_x
    if denom != 0:
        m = (n * sum_xy - sum_x * sum_y) / denom
    else:
        m = 0

    # Orientation from slope
    mag = (1 + m * m)**0.5
    orientation_x = 1 / mag
    orientation_y = m / mag
    if m < 0:
        orientation_x = -orientation_x
        orientation_y = -orientation_y
    orientation = (orientation_x, orientation_y, 0)
    orientation_angle = 57.2958 * (m / mag)  # Rough degrees
    print(f"Least-squares slope: {m:.3f}, Orientation angle: {orientation_angle:.1f}°")

    return (center_x, center_y, center_z), orientation, center_angle, center_dist

def main():
    vl53 = None
    servo = None
    interrupt = None
    current_angle = None

    try:
        print("Microcontroller booted, starting execution...")
        time.sleep(1)
        vl53, servo, current_angle, interrupt = initialize_hardware()
        print("Hardware initialized successfully")

        data, current_angle = scan(vl53, servo, interrupt, current_angle)
        edges, valid_data = detect_edges(data)

        if edges and len(edges) == 2:
            center, orientation, center_angle, center_dist = calculate_object_properties(edges, data)
            print("\nObject Properties:")
            print(f"Center Point: (x={center[0]:.1f}, y={center[1]:.1f}, z={center[2]:.1f}) mm")
            print(f"Orientation: ({orientation[0]:.3f}, {orientation[1]:.3f}, {orientation[2]:.3f})")
            print(f"Center Angle: {center_angle:.1f}°")
            print(f"Center Distance: {center_dist:.1f} mm")

            print(f"Smoothly returning servo to center angle: {center_angle:.1f}°")
            current_angle = set_servo_angle(servo, current_angle, center_angle)
            print("Servo returned to center")
            time.sleep(8)
            current_angle = set_servo_angle(servo, current_angle, 0)
        else:
            print("No object detected")
            print("Smoothly returning servo to 0°")
            current_angle = set_servo_angle(servo, current_angle, 0)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        if servo is not None:
            servo.angle = None
        if vl53 is not None:
            vl53.stop_ranging()
        if interrupt is not None:
            interrupt.deinit()
        print("Cleanup complete")

if __name__ == "__main__":
    main()

    
