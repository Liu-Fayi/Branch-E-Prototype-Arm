# Write your code here :-)
import board
import busio
import adafruit_vl53l1x
import time
import pwmio
import adafruit_motor.servo
import digitalio
import math
import struct

# Constants used for scanning, smoothing, and UART communications
ANGLE_RANGE = (0, 120)      # Range of angles to scan (degrees)
ANGLE_STEP = 1              # Step size between scans (degrees)
NUM_READINGS = 20           # Number of readings to average per angle
SMOOTH_STEP = 1             # Angular step for smooth servo transitions
SMOOTH_DELAY = 0.02         # Delay between servo increments (seconds)
INTERRUPT_TIMEOUT = 0.1     # Timeout to wait for VL53L1X interrupt (seconds)
MIN_EDGE_POINTS = 4         # Minimum number of valid points required for edge detection
UART_BAUDRATE = 9600        # Baudrate for UART communications

# Trimmed cosine table ( from 0° to 90° in 1° steps )
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

def get_cos(angle):
    """
    Return the cosine of angle (in degrees) using the table.
    For angles above 90°, return the negative cosine.
    """
    angle = max(0, min(180, angle))
    # Determine index from table – mirror if angle > 90
    idx = int(angle + 0.5) if angle <= 90 else int(180 - angle + 0.5)
    return COS_TABLE[idx] if angle <= 90 else -COS_TABLE[idx]

def get_sin(angle):
    """
    Return the sine of angle (in degrees) using the cosine table.
    Uses the identity sin(x) = cos(90 - x).
    """
    angle = max(0, min(180, angle))
    # Calculate index for sine (using cosine table offset)
    idx = int(90 - angle + 0.5) if angle <= 90 else int(angle - 90 + 0.5)
    return COS_TABLE[idx]

def initialize_hardware(interrupt_pin=board.GP18):
    """
    Initializes I2C for the VL53L1X sensor, UART, and sets up a servo.
    Also configures the interrupt pin with a pull-up resistor.
    """
    i2c = busio.I2C(scl=board.GP17, sda=board.GP16)
    i2c.try_lock()
    i2c.writeto(0x29, bytes([0x7F, 0x07]))  # Select ROI register bank
    i2c.writeto(0x29, bytes([0x09, 0x00, 0x06, 0x00]))  
    i2c.writeto(0x29, bytes([0x0A, 0x00, 0x09, 0x06])) 
    
    i2c.unlock()  
    uart = busio.UART(board.GP12, board.GP13, baudrate=UART_BAUDRATE, timeout=0.1)
    interrupt = digitalio.DigitalInOut(interrupt_pin)
    interrupt.direction = digitalio.Direction.INPUT
    interrupt.pull = digitalio.Pull.UP
    # Initialize the VL53L1X sensor
    vl53 = adafruit_vl53l1x.VL53L1X(i2c)
    vl53.distance_mode = 1  # Set sensor to short range mode, if available
    vl53.timing_budget = 50  # Set measurement timing budget
    vl53.start_ranging()
    # Setup servo on a PWM pin (using a default PWM duty setting)
    pwm = pwmio.PWMOut(board.GP0, frequency=50, duty_cycle=3**15)
    servo = adafruit_motor.servo.Servo(pwm)
    # Wait until the sensor's interrupt indicates readiness.
    if not wait_for_interrupt(vl53, interrupt):
        raise RuntimeError("VL53L1X not responding")
    return vl53, servo, 0, interrupt, uart

def wait_for_interrupt(vl53, interrupt, timeout=INTERRUPT_TIMEOUT):
    """
    Waits for an interrupt from the VL53L1X sensor within the given timeout.
    Returns True if the interrupt fires (line goes low), False otherwise.
    """
    start_time = time.monotonic()
    vl53.clear_interrupt()
    while time.monotonic() - start_time < timeout:
        if not interrupt.value:  # When interrupt is active, the value goes LOW
            return True
        time.sleep(0.001)
    return False

def wait_for_go_signal(uart):
    """
    Waits until a "go" signal (0x01) is received over UART.
    This halts execution until the signal is received.
    """
    while True:
        data = uart.read(1)
        if data and data[0] == 0x01:  # Check if the first byte is the "go" signal
            print("Received 'go' signal")
            break
        time.sleep(0.1)
        print("scaning")

def set_servo_angle(servo, current_angle, target_angle):
    """
    Smoothly moves the servo from current_angle to target_angle.
    Limits the servo movement between 0° and 180°.
    Uses SMOOTH_STEP to determine the increment and SMOOTH_DELAY between steps.
    Returns the final target angle.
    """
    target_angle = max(0, min(180, target_angle))
    if current_angle is None:
        # If there's no current angle defined, set it directly.
        servo.angle = target_angle
    else:
        # Determine the step size and direction for smooth transition.
        step = SMOOTH_STEP if abs(target_angle - current_angle) > SMOOTH_STEP else 1
        # Increment or decrement the angle in steps.
        for angle in range(int(current_angle), int(target_angle) + 1, int(step) * (1 if target_angle > current_angle else -1)):
            servo.angle = angle
            time.sleep(SMOOTH_DELAY)
    return target_angle

def read_average_distance(vl53, interrupt, num_readings=NUM_READINGS):
    """
    Takes several distance measurements from the VL53L1X sensor,
    averages those that are within a valid range (10 to 4000 mm), and returns the result.
    Returns None if no valid reading is obtained.
    """
    total = count = 0
    for _ in range(num_readings):
        if wait_for_interrupt(vl53, interrupt):
            distance = vl53.distance
            if distance is not None:
                # Convert distance from cm to mm
                distance *= 10
                if 10 < distance < 4000:
                    total += distance
                    count += 1
            # Clear sensor interrupt for the next reading.
            vl53.clear_interrupt()
    return total / count if count else None

def scan(vl53, servo, interrupt, current_angle):
    """
    Scans through a range of angles defined by ANGLE_RANGE and ANGLE_STEP.
    At each angle, the servo is moved smoothly to the target position,
    and the sensor reading is averaged.
    Returns a list of tuples (angle, measured_distance) and the final servo angle.
    """
    angles = range(ANGLE_RANGE[0], ANGLE_RANGE[1] + 1, ANGLE_STEP)
    data = []
    for angle in angles:
        current_angle = set_servo_angle(servo, current_angle, angle)
        distance = read_average_distance(vl53, interrupt)
        if distance != None:
            x = distance * get_sin(current_angle)
            y = distance * get_cos(current_angle)
            print(distance)
        #print(current_angle, distance)
        data.append((angle, distance))
        time.sleep(.1)
    return data, current_angle

def detect_edges(data):
    """
    Analyzes the scanned data to detect edges based on the slopes between successive readings.
    Returns a tuple: (edge_list, valid_data) where edge_list contains detected edges,
    and valid_data contains the data points with non-None distances.
    """
    valid_data = [(a, d) for a, d in data if d is not None]
    if len(valid_data) < MIN_EDGE_POINTS:
        return [], valid_data

    angles, distances = zip(*valid_data)
    # Compute the slope between successive points
    slopes = [0] + [(distances[i] - distances[i-1]) / ANGLE_STEP for i in range(1, len(distances))]
    # Smooth slopes by averaging each point with its neighbors
    smoothed = slopes[:1] + [(slopes[i-1] + slopes[i] + slopes[i+1]) / 3 for i in range(1, len(slopes) - 1)] + slopes[-1:]
    
    # Find the minimum and maximum slopes and their indices
    min_slope, max_slope = float('inf'), float('-inf')
    min_idx = max_idx = None
    for i in range(1, len(smoothed) - 1):
        if smoothed[i] < min_slope:
            min_slope, min_idx = smoothed[i], i
        if smoothed[i] > max_slope:
            max_slope, max_idx = smoothed[i], i

    # Return two edge tuples if valid indices are found and in correct order.
    return ([(angles[min_idx-1], distances[min_idx-1], angles[min_idx], distances[min_idx]),
             (angles[max_idx-1], distances[max_idx-1], angles[max_idx], distances[max_idx])]
            if min_idx and max_idx and min_idx < max_idx else []), valid_data

def calculate_object_properties(edges, data):
    """
    Given detected edge pairs from the scan, calculates object properties:
    center position (in polar coordinates converted to Cartesian) and orientation.
    Returns a tuple of (center coordinates, orientation vector, center_angle, average distance).
    """
    if len(edges) < 2:
        return None, None, None, None

    valid_data = [(a, d) for a, d in data if d is not None]
    start_idx = next(i for i, (a, _) in enumerate(valid_data) if a == edges[0][2])
    end_idx = next(i for i, (a, _) in enumerate(valid_data) if a == edges[1][0])

    sum_angle_dist = sum_dist = 0
    distances = [d for _, d in valid_data[start_idx:end_idx+1]]
    for angle, dist in valid_data[start_idx:end_idx+1]:
        sum_angle_dist += angle * dist
        sum_dist += dist
    center_angle = sum_angle_dist / sum_dist if sum_dist else (edges[0][2] + edges[1][0]) / 2
    center_dist = sum(distances) / len(distances)

    # Convert polar center to Cartesian coordinates
    center_x = center_dist * get_cos(center_angle)
    center_y = center_dist * get_sin(center_angle)

    n = end_idx - start_idx + 1
    sum_x = sum_y = sum_xy = sum_xx = 0
    for angle, dist in valid_data[start_idx:end_idx+1]:
        x = dist * get_cos(angle)
        y = dist * get_sin(angle)
        sum_x += x
        sum_y += y
        sum_xy += x * y
        sum_xx += x * x

    denom = n * sum_xx - sum_x * sum_x
    m = (n * sum_xy - sum_x * sum_y) / denom if denom else 0
    mag = (1 + m * m)**0.5
    orientation_x = 1 / mag
    orientation_y = m / mag if m >= 0 else -m / mag
    orientation_x = -orientation_x if m < 0 else orientation_x

    return (center_x, center_y, 0), (orientation_x, orientation_y, 0), center_angle, center_dist

def send_uart_data(uart, center, orientation):
    """
    Sends object properties (center and orientation) over UART as six floats.
    Uses struct.pack to format the binary packet.
    """
    uart.write(struct.pack('ffffff', *center, *orientation))
    print("Sent UART data")

def main():
    vl53, servo, current_angle, interrupt, uart = None, None, None, None, None

    try:
        # Initialize hardware components (sensor, servo, interrupt, uart)
        vl53, servo, current_angle, interrupt, uart = initialize_hardware()
        # Wait for an external "go" signal to start scanning over UART
        wait_for_go_signal(uart)

        # Scan over the defined angle range and get all the readings
        data, current_angle = scan(vl53, servo, interrupt, current_angle)
        # Detect edges in the measured distance data
        edges, valid_data = detect_edges(data)

        # If two edges are detected, calculate object properties and send over UART.
        if len(edges) == 2:
            center, orientation, center_angle, center_dist = calculate_object_properties(edges, data)
            send_uart_data(uart, center, orientation)
            print(center, orientation, center_dist)
            print(math.atan2(orientation[1], orientation[0]))
            # Move servo to center angle, wait, then return to zero
            current_angle = set_servo_angle(servo, current_angle, center_angle)
            time.sleep(8)
            current_angle = set_servo_angle(servo, current_angle, 0)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean-up resources from sensor, interrupt, UART and release servo control.
        if servo: 
            servo.angle = None
        if vl53: 
            vl53.stop_ranging()
        if interrupt: 
            interrupt.deinit()
        if uart: 
            uart.deinit()

if __name__ == "__main__":
    main()