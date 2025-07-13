import pybullet as p
import pybullet_data
import time
import math
from src.sim.robot import Robot
from src.sim.simulation import BulletSim
from src.sim.model_obj import ModelObj
import matplotlib.pyplot as plt
from src.perception.crop import AABBCropper
from PIL import Image
import torch
import numpy as np
import clip

if __name__ == "__main__":
    # Load the CLIP model
    device = "mps" if torch.backends.mps.is_available() else "cpu"
    model, preprocess = clip.load("ViT-B/32", device=device)

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
            orientation=p.getQuaternionFromEuler([0, 0, math.pi/2]),
            position=[0.7, -0.3, 0.03],
            scale=0.1,
            grasp_offset=0.03
        )
    ]
    sim.register_objects(objs)

    goals = [
        ModelObj(
            name="plate",
            path="029_plate.urdf",
            position=[0.0, 0.7, 0.01],
            scale=0.1
        ),
    ]
    sim.register_objects(goals)

    img = robot.get_ee_camera_image()
    cropper = AABBCropper(sim, robot, img, objs)
    cropped_images, boxes_img = cropper.crop_all_objects()
    image_feature_list = []

    # Encode all images
    for obj_img in cropped_images:
        image = preprocess(
            Image.fromarray(obj_img)
        ).unsqueeze(0).to(device)
        with torch.no_grad():
            image_features = model.encode_image(image)
            image_features = image_features.cpu()
            image_feature_list.append(image_features)

    # Tokenize user input
    user_input = input("Enter a query: ")
    # user_input = "a banana"
    labels = [user_input]
    text = clip.tokenize(labels).to(device)

    # Encode the image and text
    with torch.no_grad():
        text_features = model.encode_text(text).cpu()

    # Compute the cosine similarity between the image and text features
    similarities = []
    for i, image_feature in enumerate(image_feature_list):
        similarity = torch.nn.functional.cosine_similarity(image_feature, text_features, dim=1).item()
        print(f"Image {i} similarity: {similarity}")
        similarities.append(similarity)

    # Display the best match
    inferred_obj_id = np.argmax(similarities)
    print(f"Best match: {objs[inferred_obj_id].name}")

    # Move the robot to the best match
    des_joints_A = robot.ik(
        objs[inferred_obj_id].position[0], 
        objs[inferred_obj_id].position[1],
        objs[inferred_obj_id].position[2] + 0.3 # Small z offset
    )

    print("Position Control")
    robot.open_gripper()
    robot.position_control(des_joints_A, max_steps=1000)

    des_joints_B = robot.ik(
        objs[inferred_obj_id].position[0], 
        objs[inferred_obj_id].position[1],
        objs[inferred_obj_id].position[2] + objs[inferred_obj_id].grasp_offset + 0.03 # Small z offset
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
        goals[0].position[2] + objs[inferred_obj_id].grasp_offset  # Drop the object above the goal if it's large.
    ) # Small z offset

    print("Position Control")
    robot.position_control(des_joints, max_velocity=1.3, max_steps=300)
    robot.open_gripper()

    # # Sim loop
    for i in range(50):
        p.stepSimulation()
        time.sleep(1./240.)
