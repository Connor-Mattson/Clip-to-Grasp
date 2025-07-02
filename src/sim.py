import pybullet as p
import pybullet_data
import time
import math

class Robot:
    def __init__(self, robot_id, ee_link_index=11, joint_range=[0, 7], gripper_joints=[9, 11]):
        self.id = robot_id
        self.ee_ind = ee_link_index
        self.joints = range(joint_range[0], joint_range[1])
        self.gripper_joints = range(gripper_joints[0], gripper_joints[1])
        self.gripper_open = [0.08, 0.08]
        self.gripper_close = [0.0, 0.0]

        self.lower_limits = []
        self.upper_limits = []
        self.joint_ranges = []
        self.rest_poses = []

        for joint_index in self.joints:
            info = p.getJointInfo(self.id, joint_index)
            ll, ul = info[8], info[9]
            self.lower_limits.append(ll)
            self.upper_limits.append(ul)
            self.joint_ranges.append(ul - ll)
            # A mild rest pose bias: elbow bent
            self.rest_poses.append((ll + ul) / 2.0 if ll < ul else 0.0)

    def ik(self, x, y, z):
        target_orn = p.getQuaternionFromEuler([math.pi, 0, 0])
        target_pos = [x, y, z]

        joint_poses = p.calculateInverseKinematics(
            bodyUniqueId=self.id,
            endEffectorLinkIndex=self.ee_ind,
            targetPosition=target_pos,
            targetOrientation=target_orn,
            lowerLimits=self.lower_limits,
            upperLimits=self.upper_limits,
            jointRanges=self.joint_ranges,
            restPoses=self.rest_poses,
            maxNumIterations=100,
            residualThreshold=1e-4,
        )

        joint_poses = joint_poses[:len(self.joints)]

        # Check how close the EE actually gets
        ee_state = p.getLinkState(self.id, self.ee_ind)
        actual_pos = ee_state[4]
        error = math.dist(actual_pos, target_pos)

        if error > 0.02:
            print(f"[WARN] IK error too large ({error:.3f}m) — pose may not be reachable or orientation constraint too strict.")

        return joint_poses

    def position_control(self, joint_poses):
        for i, joint_index in enumerate(self.joints): 
            p.setJointMotorControl2(self.id, joint_index, p.POSITION_CONTROL, targetPosition=joint_poses[i])

    def open_gripper(self):
        for i, joint_value in zip(self.gripper_joints, self.gripper_open):
            p.setJointMotorControl2(
                bodyIndex=self.id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_value,
                force=10
            )
    
    def close_gripper(self):
        for i, joint_value in zip(self.gripper_joints, self.gripper_close):
            p.setJointMotorControl2(
                bodyIndex=self.id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_value,
                force=10
            )

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

# Load in an example object
# object_start = [0.7, 0.2, 0.05]
object_start = [0.3, 0.2, 0.05]

object_id = p.loadURDF("cube_small.urdf", object_start)

num_joints = p.getNumJoints(robot_id)
print(f"Robot has {num_joints} joints.")

print("Calculate IK")
des_joints = robot.ik(object_start[0], object_start[1], object_start[2] + 0.095) # Small z offset

print("Position Control")
robot.position_control(des_joints)
robot.open_gripper()

print("Simulation Steps")
# # Sim loop
for i in range(100):
    p.stepSimulation()
    time.sleep(1./240.)

robot.close_gripper()
print("Simulation Steps")
# # Sim loop
for i in range(30):
    p.stepSimulation()
    time.sleep(1./240.)

goal_pos = [0.7, 0.2, 0.2]
des_joints = robot.ik(goal_pos[0], goal_pos[1], goal_pos[2]) # Small z offset

print("Position Control")
robot.position_control(des_joints)
print("Simulation Steps")
# # Sim loop
for i in range(100):
    p.stepSimulation()
    time.sleep(1./240.)

robot.open_gripper()
print("Simulation Steps")
# # Sim loop
for i in range(50):
    p.stepSimulation()
    time.sleep(1./240.)