import sys
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

# CLIP cosine similarity the best crop must reach to count as a match. With the "a photo of ..." prompt,
# queries naming an object in this scene scored >= 0.272 and most unrelated queries <= 0.263, but queries
# for similar-looking objects (e.g. "a hammer") can reach ~0.27, so this only rejects clearly unrelated ones.
MIN_SIMILARITY = 0.265
# Lowest grasp target (m) that keeps the fingertips off the ground plane
MIN_GRASP_HEIGHT = 0.02
# Gap (m) between the bottom of the held object and the top of the plate when released
PLACE_CLEARANCE = 0.01
# Joint tolerance (rad) for considering a motion complete
JOINT_TOLERANCE = 0.01
# Grasp at most this far (m) below an object's top so the palm of the hand clears it
MAX_GRASP_DEPTH = 0.04


def grasp_point(sim, obj):
    """Top-down grasp target from the object's current simulated pose (not its spawn position)."""
    (x, y, _), _ = p.getBasePositionAndOrientation(obj.oid)
    box_min, box_max = sim.get_object_aabb(obj.oid)
    z = max((box_min[2] + box_max[2]) / 2, box_max[2] - MAX_GRASP_DEPTH) + obj.grasp_offset
    return [x, y, max(z, MIN_GRASP_HEIGHT)]


def move_straight(robot, start, end, waypoints=10, **control_kwargs):
    """Move the EE along a straight line through IK waypoints. A single joint-space move swings the
    hand in an arc, which can sweep it into the object being approached."""
    for t in np.linspace(0, 1, waypoints + 1)[1:]:
        waypoint = [s + t * (e - s) for s, e in zip(start, end)]
        robot.position_control(robot.ik(*waypoint), **control_kwargs)


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
            scale=0.1
        ),
        ModelObj(
            name="banana",
            path="011_banana.urdf",
            position=[0.7, 0.1, 0.01],
            orientation=p.getQuaternionFromEuler([0, 0, math.pi/2]),
            scale=0.1
        ),
        ModelObj(
            name="soup",
            path="005_tomato_soup_can.urdf",
            position=[0.7, -0.1, 0.05],
            # At 0.1 the can is ~7.7cm wide, too close to the gripper's ~8cm opening to grasp top-down
            scale=0.085
        ),
        ModelObj(
            name="mug",
            path="025_mug.urdf",
            orientation=p.getQuaternionFromEuler([0, 0, math.pi/2]),
            position=[0.7, -0.3, 0.03],
            scale=0.1
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

    # Let the objects settle under gravity before perceiving the scene
    for i in range(240):
        p.stepSimulation()

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
    labels = [f"a photo of {user_input}"]
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

    # Display the best match, or stop if nothing in the scene matches the query
    inferred_obj_id = np.argmax(similarities)
    if similarities[inferred_obj_id] < MIN_SIMILARITY:
        print(f"No object matches '{user_input}' "
              f"(best similarity {similarities[inferred_obj_id]:.3f} < {MIN_SIMILARITY})")
        sim.close()
        sys.exit(1)
    target = objs[inferred_obj_id]
    print(f"Best match: {target.name}")

    # Grasp from where the object actually is now
    grasp = grasp_point(sim, target)
    grasp_above_bottom = grasp[2] - sim.get_object_aabb(target.oid)[0][2]

    # Move above the best match
    above_grasp = [grasp[0], grasp[1], grasp[2] + 0.3]

    print("Position Control")
    robot.open_gripper()
    robot.position_control(robot.ik(*above_grasp), max_steps=1000, tolerance=JOINT_TOLERANCE)

    # Descend straight down onto the object
    print("Position Control")
    move_straight(robot, above_grasp, grasp, max_steps=200, max_velocity=1.1, tolerance=JOINT_TOLERANCE)

    robot.close_gripper()
    # Sim loop
    for i in range(50):
        p.stepSimulation()
        time.sleep(1./240.)

    print("Position Control")
    move_straight(robot, grasp, above_grasp, max_steps=200, max_velocity=1.1, tolerance=JOINT_TOLERANCE)

    # Release so the bottom of the object sits just above the top of the plate
    (goal_x, goal_y, _), _ = p.getBasePositionAndOrientation(goals[0].oid)
    place_z = sim.get_object_aabb(goals[0].oid)[1][2] + grasp_above_bottom + PLACE_CLEARANCE
    place = [goal_x, goal_y, place_z]
    above_place = [goal_x, goal_y, place_z + 0.2]

    print("Position Control")
    robot.position_control(robot.ik(*above_place), max_steps=1000, tolerance=JOINT_TOLERANCE)

    # Decend to the goal
    print("Position Control")
    move_straight(robot, above_place, place, max_steps=200, max_velocity=1.3, tolerance=JOINT_TOLERANCE)
    robot.open_gripper()

    # # Sim loop
    for i in range(50):
        p.stepSimulation()
        time.sleep(1./240.)
