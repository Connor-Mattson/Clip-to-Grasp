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
        position=[0.7, 0.3, 0.05],
        scale=0.07
    ),
    ModelObj(
        name="banana",
        path="011_banana.urdf",
        position=[0.7, 0.1, 0.05],
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
        position=[0.7, -0.3, 0.05],
        scale=0.07
    )
]
sim.register_objects(objs)

cam_info = p.getDebugVisualizerCamera()
print(cam_info)