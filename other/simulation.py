import pybullet as p
import pybullet_data
import time
import numpy as np
import math

# Connect to PyBullet with GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

# Arm parameters (in cm)
L1 = 16.5      # Upper arm length
L2 = 12.647    # Forearm length
CLAW_LENGTH = 3  # Claw length (approximate, adjust as needed)

# Create a 5-DOF arm: base (z), link1 (upper arm), link2 (forearm), wrist, claw
base_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.15, 0.15, 0.05])
link1_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.01, height=L1/100)
link2_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.01, height=L2/100)
wrist_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.01, height=0.05)
claw_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, CLAW_LENGTH/200, 0.02])

arm_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=base_id,
    basePosition=[0, 0, 0],
    linkMasses=[1, 1, 0.5, 0.2],
    linkCollisionShapeIndices=[link1_id, link2_id, wrist_id, claw_id],
    linkVisualShapeIndices=[-1, -1, -1, -1],
    linkPositions=[[0, 0, L1/200], [0, 0, L2/200], [0, 0, 0.05/2], [0, 0, CLAW_LENGTH/200]],
    linkOrientations=[[0, 0, 0, 1]] * 4,
    linkInertialFramePositions=[[0, 0, 0]] * 4,
    linkInertialFrameOrientations=[[0, 0, 0, 1]] * 4,
    linkParentIndices=[0, 1, 2, 3],
    linkJointTypes=[p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE],
    linkJointAxis=[[0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0]]  # Y-axis rotation
)

# Reset initial joint states
for i in range(4):
    p.resetJointState(arm_id, i, 0)

# Trajectory from your main() with TEST_COORDS and wrist angle
TEST_COORDS = (10.0, 5.0, 0.0, 45.0)  # (x, y, z, wrist_angle in degrees)
X_OFF, Y_OFF, ANGLE_OFF = 0, 0, 0

def transform_sensor_to_arm(x, y, z, angle):
    theta = math.radians(angle)
    x_arm = X_OFF + x * math.cos(theta) - y * math.sin(theta)
    y_arm = Y_OFF + x * math.sin(theta) + y * math.cos(theta)
    angle_arm = angle + ANGLE_OFF
    return x_arm, y_arm, z, angle_arm

# Define sequence from main()
t_seq = [0, 2, 3, 4, 5, 6]  # Extended for final open claw
x0, y0, z0, angle0 = TEST_COORDS
ax, ay, az, a_angle = transform_sensor_to_arm(x0, y0, z0, angle0)
x_seq = [0, ax, ax, ax, ax, 0]  # Start at (0,0), move to ax,ay, then back
y_seq = [0, ay, ay, ay, ay, 0]
z_seq = [0, 10, 3, 10, 10, 10]  # z sequence from main()
wrist_seq = [0, a_angle, a_angle, a_angle, a_angle, 0]  # Wrist angle in degrees
claw_seq = [0, 0, 70, 0, 0, 0]  # Claw: 0 (open), 70 (closed)

dt = 0.05
time_a = np.arange(0, t_seq[-1] + dt, dt)
n_steps = len(time_a)

# Interpolate trajectory
x_traj = np.interp(time_a, t_seq, x_seq)
y_traj = np.interp(time_a, t_seq, y_seq)
z_traj = np.interp(time_a, t_seq, z_seq)
wrist_traj = np.interp(time_a, t_seq, wrist_seq)
claw_traj = np.interp(time_a, t_seq, claw_seq)

# IK solver from your IKSolver
def ik_solver(x, y, L1, L2):
    t = math.atan2(y, x)
    u1 = (L1**2 + L2**2 - x**2 - y**2) / (2 * L1 * L2)
    u1 = np.clip(u1, -1, 1)
    u1 = math.acos(u1)
    u2 = (L1**2 + x**2 + y**2 - L2**2) / (2 * L1 * math.sqrt(x**2 + y**2))
    u2 = np.clip(u2, -1, 1)
    u2 = math.acos(u2)
    t1 = t - u2  # Base angle
    t2 = math.pi - u1  # Elbow angle
    return t1, t2

# Markers for visualization
marker_radius = 0.05
elbow_marker = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_SPHERE, radius=marker_radius),
    baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=marker_radius, rgbaColor=[0, 0, 1, 0.8])
)
end_marker = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_SPHERE, radius=marker_radius),
    baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=marker_radius, rgbaColor=[1, 0, 0, 0.8])
)

# Dot trail storage
elbow_dots = []
end_dots = []

p.setRealTimeSimulation(0)
for i in range(n_steps):
    x_m = x_traj[i] / 100
    y_m = y_traj[i] / 100
    z_m = z_traj[i] / 100
    wrist_angle = math.radians(wrist_traj[i])
    claw_angle = math.radians(claw_traj[i])

    # Compute IK
    theta1, theta2 = ik_solver(x_traj[i], y_traj[i], L1, L2)

    # Update arm state
    p.resetBasePositionAndOrientation(arm_id, [0, 0, z_m], [0, 0, 0, 1])
    p.setJointMotorControl2(arm_id, 0, p.POSITION_CONTROL, targetPosition=theta1)
    p.setJointMotorControl2(arm_id, 1, p.POSITION_CONTROL, targetPosition=theta2)
    p.setJointMotorControl2(arm_id, 2, p.POSITION_CONTROL, targetPosition=wrist_angle)
    p.setJointMotorControl2(arm_id, 3, p.POSITION_CONTROL, targetPosition=claw_angle)

    # Step simulation
    p.stepSimulation()

    # Get positions
    link1_state = p.getLinkState(arm_id, 0)  # Elbow
    link3_state = p.getLinkState(arm_id, 2)  # Wrist (end effector base)
    elbow_pos = link1_state[0]
    wrist_pos = link3_state[0]
    end_pos = p.getLinkState(arm_id, 3)[0]  # Claw tip (end effector)

    # Update markers
    p.resetBasePositionAndOrientation(elbow_marker, elbow_pos, [0, 0, 0, 1])
    p.resetBasePositionAndOrientation(end_marker, end_pos, [0, 0, 0, 1])

    # Add dot trail every few steps for clarity
    if i % 5 == 0:  # Reduce dot density
        elbow_dots.append(p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_SPHERE, radius=0.02),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[0, 0, 1, 0.5])
        ))
        p.resetBasePositionAndOrientation(elbow_dots[-1], elbow_pos, [0, 0, 0, 1])
        end_dots.append(p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_SPHERE, radius=0.02),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 0.5])
        ))
        p.resetBasePositionAndOrientation(end_dots[-1], end_pos, [0, 0, 0, 1])

    time.sleep(dt)  # Slower for better visibility

print("Simulation complete. Close the window to exit.")
while p.isConnected():
    time.sleep(0.1)

p.disconnect()