import pybullet as p
import pybullet_data
import time
import math
from src.sim.robot import Robot
from src.sim.simulation import BulletSim
from src.sim.model_obj import ModelObj
import matplotlib.pyplot as plt
from src.perception.crop import AABBCropper

# Connect to GUI
sim = BulletSim()
sim.set_gravity(0, 0, -9.81)
sim.enable_auxiliary_camera()

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
initial_joint_positions = [-0.036, 0.175, 0.0574, -0.469, -0.0169, 0.687, 0.791]
for i, joint_pos in enumerate(initial_joint_positions):
    p.resetJointState(robot_id, i, joint_pos)

# Add our models to the simulators search path
sim.extend_search_path("models/ycb")

# Load in an example object
objs = [
    ModelObj(
        name="apple",
        path="013_apple.urdf",
        position=[0.7, 0.3, 0.025],
        scale=0.1,
        grasp_offset=-0.01
    ),
    ModelObj(
        name="banana",
        path="011_banana.urdf",
        position=[0.7, 0.1, 0.01],
        orientation=p.getQuaternionFromEuler([0, 0, math.pi/2]),
        scale=0.1,
        grasp_offset=0.01
    ),
    ModelObj(
        name="soup",
        path="005_tomato_soup_can.urdf",
        position=[0.7, -0.1, 0.05],
        scale=0.1,
        grasp_offset=0.04
    ),
    ModelObj(
        name="mug",
        path="025_mug.urdf",
        position=[0.7, -0.3, 0.03],
        scale=0.1,
        grasp_offset=0.03
    )
]
sim.register_objects(objs)

img = robot.get_ee_camera_image()

# TODO: Extract the object bounding boxesfrom the image
cropper = AABBCropper(sim, robot, img, objs)
cropped_images, boxes_img = cropper.crop_all_objects()

plt.imshow(boxes_img)
plt.show()

# Go through each cropped image and show it
for i, img in enumerate(cropped_images):
    plt.imshow(img)
    plt.show()