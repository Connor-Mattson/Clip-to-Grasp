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

        # Set up proper rest poses - especially important for elbow joint
        good_rest_poses = [0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0]  # Elbow bent naturally
        
        for i, joint_index in enumerate(self.joints):
            info = p.getJointInfo(self.id, joint_index)
            ll, ul = info[8], info[9]
            self.lower_limits.append(ll)
            self.upper_limits.append(ul)
            self.joint_ranges.append(ul - ll)
            self.rest_poses.append(good_rest_poses[i])
            
        print(f"Robot initialized with proper joint limits and rest poses.")

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
                
        # Check if any joint commands are outside limits (should not happen with proper config)
        for i, (jp, ll, ul) in enumerate(zip(joint_poses, self.lower_limits, self.upper_limits)):
            if jp < ll - 0.001 or jp > ul + 0.001:  # Small tolerance for floating point
                joint_name = p.getJointInfo(self.id, list(self.joints)[i])[1].decode('utf-8')
                print(f"WARNING: Joint {list(self.joints)[i]} ({joint_name}) = {jp:.3f} outside limits [{ll:.3f}, {ul:.3f}]")

        # Check how close the EE actually gets
        ee_state = p.getLinkState(self.id, self.ee_ind)
        actual_pos = ee_state[4]
        error = math.dist(actual_pos, target_pos)

        if error > 0.02:
            print(f"[WARN] IK error too large ({error:.3f}m) — pose may not be reachable or orientation constraint too strict.")

        return joint_poses

    def position_control(self, joint_poses):
        # Apply position control with stronger motor parameters
        for i, joint_index in enumerate(self.joints): 
            p.setJointMotorControl2(
                bodyIndex=self.id,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_poses[i],
                # positionGain=0.1,      # P gain for responsiveness
                # velocityGain=0.01,     # D gain for stability  
                # force=200,             # Increased max force
                # maxVelocity=2.0        # Increased max velocity
            )
            
        # Wait for joints to reach target positions
        max_steps = 1000  # Increased timeout
        for step in range(max_steps):
            p.stepSimulation()
            time.sleep(1./240.)
            
            # Check convergence every 50 steps
            if step % 50 == 0:
                all_converged = True
                for i, joint_index in enumerate(self.joints):
                    current_pos = p.getJointState(self.id, joint_index)[0]
                    target_pos = joint_poses[i]
                    error = abs(current_pos - target_pos)
                    
                    if error > 0.05:  # 0.05 radian tolerance
                        all_converged = False
                
                if all_converged:
                    print(f"All joints converged after {step} steps")
                    break
        
        if step >= max_steps - 1:
            print(f"Warning: Not all joints converged within {max_steps} steps")
            # Print final errors
            for i, joint_index in enumerate(self.joints):
                current_pos = p.getJointState(self.id, joint_index)[0]
                target_pos = joint_poses[i]
                error = abs(current_pos - target_pos)
                print(f"  Final Joint {joint_index}: {current_pos:.3f} -> {target_pos:.3f} (error: {error:.3f})")

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