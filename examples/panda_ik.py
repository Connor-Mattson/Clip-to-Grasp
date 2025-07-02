import pybullet as p
import pybullet_data
import time
import math
from src.sim.robot import Robot

# Connect to GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # load URDFs

# Set up simulation
p.setGravity(0, 0, -9.81)
plane_id = p.loadURDF("plane.urdf")

# Load Franka Panda robot
start_pos = [0, 0, 0]
start_ori = p.getQuaternionFromEuler([0, 0, 0])
robot_id = p.loadURDF("franka_panda/panda.urdf", start_pos, start_ori, useFixedBase=True)

# Create robot class
robot = Robot(robot_id, 11, [0, 7], [9, 11])

# Set robot to proper initial configuration (elbow bent)
print("Setting robot to proper initial configuration...")
initial_joint_positions = [0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0]
for i, joint_pos in enumerate(initial_joint_positions):
    p.resetJointState(robot_id, i, joint_pos)

# Load in an example object  
object_start = [0.7, 0.2, 0.05]  # Reachable position

object_id = p.loadURDF("cube_small.urdf", object_start)

num_joints = p.getNumJoints(robot_id)
print(f"Robot has {num_joints} joints.")

print("Calculate IK")
des_joints = robot.ik(object_start[0], object_start[1], object_start[2] - 0.03) # Small z offset

print("Position Control")
robot.open_gripper()
robot.position_control(des_joints)

robot.close_gripper()
print("Simulation Steps")
# # Sim loop
for i in range(20):
    p.stepSimulation()
    time.sleep(1./240.)

goal_pos = [0.7, 0.2, 0.2]
des_joints = robot.ik(goal_pos[0], goal_pos[1], goal_pos[2]) # Small z offset

print("Position Control")
robot.position_control(des_joints)
print("Simulation Steps")

robot.open_gripper()
print("Simulation Steps")
# # Sim loop
for i in range(50):
    p.stepSimulation()
    time.sleep(1./240.)