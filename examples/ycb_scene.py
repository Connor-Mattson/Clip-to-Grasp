import pybullet as p
import pybullet_data
import time
import math
from src.sim.robot import Robot
from src.sim.simulation import BulletSim
from src.sim.model_obj import ModelObj

# Connect to GUI
sim = BulletSim()
sim.set_gravity(0, 0, -9.81)

# Shift the camera to a better position
p.resetDebugVisualizerCamera(
    cameraDistance=1.5129878520965576,
    cameraYaw=93.12030792236328,
    cameraPitch=-10.921874046325684,
    cameraTargetPosition=[0.0, 0.0, 0.0]
)

# Load Franka Panda robot
start_pos = [0, 0, 0]
start_ori = p.getQuaternionFromEuler([0, 0, 0])
robot_id = p.loadURDF("franka_panda/panda.urdf", start_pos, start_ori, useFixedBase=True)

# Create robot class
# TODO: Have this class consume the simulation calls + pass only the name of the robot
robot = Robot(robot_id, 11, [0, 7], [9, 11])

# TODO: Abstract away to the Robot class
# Set robot to proper initial configuration (elbow bent)
print("Setting robot to proper initial configuration...")
initial_joint_positions = [0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0]
for i, joint_pos in enumerate(initial_joint_positions):
    p.resetJointState(robot_id, i, joint_pos)

# Add our models to the simulators search path
sim.extend_search_path("models/ycb")

# Load in an example object
objs = [
    ModelObj(
        name="apple",
        path="013_apple.urdf",
        position=[0.7, 0.3, 0.03],
        scale=0.07
    ),
    ModelObj(
        name="banana",
        path="011_banana.urdf",
        position=[0.7, 0.1, 0.02],
        scale=0.07
    ),
    ModelObj(
        name="soup",
        path="005_tomato_soup_can.urdf",
        position=[0.7, -0.1, 0.05],
        scale=0.07
    ),
    ModelObj(
        name="mug",
        path="025_mug.urdf",
        position=[0.7, -0.3, 0.03],
        scale=0.07
    )
]
sim.register_objects(objs)

goals = [
    ModelObj(
        name="plate",
        path="029_plate.urdf",
        position=[0.0, 0.7, 0.03],
        scale=0.07
    ),
]
sim.register_objects(goals)

OBJECT_TO_GRASP = 3

des_joints_A = robot.ik(
    objs[OBJECT_TO_GRASP].position[0], 
    objs[OBJECT_TO_GRASP].position[1],
    objs[OBJECT_TO_GRASP].position[2] + 0.2 # Small z offset
)

print("Position Control")
robot.open_gripper()
robot.position_control(des_joints_A, max_steps=1000)

des_joints_B = robot.ik(
    objs[OBJECT_TO_GRASP].position[0], 
    objs[OBJECT_TO_GRASP].position[1],
    objs[OBJECT_TO_GRASP].position[2] - 0.00 # Small z offset
)

print("Position Control")
robot.position_control(des_joints_B, max_steps=800, max_velocity=1.1)

robot.close_gripper()
# Sim loop
for i in range(50):
    p.stepSimulation()
    time.sleep(1./240.)

print("Position Control")
robot.position_control(des_joints_A, max_steps=800, max_velocity=1.1)  

des_joints = robot.ik(
    goals[0].position[0], 
    goals[0].position[1], 
    goals[0].position[2] + 0.2
) # Small z offset

print("Position Control")
robot.position_control(des_joints, max_steps=1000)

# Decend to the goal
des_joints = robot.ik(
    goals[0].position[0], 
    goals[0].position[1], 
    goals[0].position[2] + 0.05
) # Small z offset

print("Position Control")
robot.position_control(des_joints, max_velocity=1.3, max_steps=300)


robot.open_gripper()

# # Sim loop
for i in range(50):
    p.stepSimulation()
    time.sleep(1./240.)