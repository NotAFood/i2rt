from i2rt.robots.get_robot import get_yam_robot
import numpy as np
#import pyroki_snippets as pks
import time
from i2rt.robots.motor_chain_robot import *
""" Ported to YamKit"""
# Get a robot instance
joint_pos0 = None
joint_pos1 = None
currentBot="can1"
def setUpInZeroG(robot1,robot2):
    startRobotInZeroG(robot1,robot2)
    #can also run by port using below line
    #os.system("python i2rt/robots/motor_chain_robot.py --channel " + canPort  + " --gripper_type yam_compact_small")
def getURDF():
    yam_urdf_path = "/home/yam/i2rt/robot_models/yam/yam.urdf"
    
    # Check if URDF file exists
    if not os.path.exists(yam_urdf_path):
        print(f"Error: URDF file not found at {yam_urdf_path}")
    # Load URDF content directly from file
    with open(yam_urdf_path, 'r') as f:
        urdf_content = f.read()
    
    # Update URDF paths to use absolute paths for assets
    asset_dir = "/home/yam/i2rt/robot_models/yam/assets"
    urdf_content = urdf_content.replace("package://assets", asset_dir)
    
    # Parse URDF using yourdfpy - use io.StringIO to create file-like object from string
    urdf_file_like = io.StringIO(urdf_content)
    urdf = yourdfpy.URDF.load(urdf_file_like)
    return urdf

def moveToPosSlow(robot, target_pos = np.array([0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), duration=2.0):
    """Move to position  over a specified duration."""
    if len(target_pos) == 6:
        print("passed in:" ,target_pos)
        target_pos = np.append(target_pos,0.0)
        print("sent to:",target_pos)
    robot.move_joints(target_pos, time_interval_s=duration)

def moveToPosVelo(robot, target_pos = np.array([0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), max_velocity=0.1):
    """Move to position with velocity control"""
    if len(target_pos) == 6:
        print("passed in:" ,target_pos)
        target_pos = np.append(target_pos,0.0)
        print("sent to:",target_pos)
    # Use very low velocities for slow movement
    slow_velocities = np.array([max_velocity] * 7)
    robot.command_joint_state({
        "pos": target_pos,
        "vel": slow_velocities,
        "kp": np.array([5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0]),  # Lower kp for softer control
        "kd": np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])   # Lower kd for less damping
    })

def moveToPosTrajectory(robot, target_pos = np.array([0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), duration=10.0):
    """Move to position using trajectory interpolation for slow movement.
    Args:
        robot: the robot to command 
        target_pos: Target joint positions
        duration 
    """
    if len(target_pos) == 6:
        print("passed in:" ,target_pos)
        target_pos = np.append(target_pos,0.0)
        print("sent to:",target_pos)
    
    current_pos = robot.get_joint_pos()
    
    # Calculate number of steps based on duration (50 Hz control rate)
    steps = int(duration * 50)  # 50 Hz = 50 steps per second
    step_time = duration / steps
    
    print(f"Moving from {current_pos} to {target_pos} over {duration}s ({steps} steps)")
    
    for i in range(steps + 1):
        alpha = i / steps  # Interpolation factor
        interpolated_pos = (1 - alpha) * current_pos + alpha * target_pos
        robot.command_joint_pos(interpolated_pos)
        time.sleep(step_time)

def moveToPos(robot, target_pos = np.array([0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), max_velocity=2):
    """Move to position with velocity-controlled movement.
    
    Args:
        target_pos: Target joint positions
        max_velocity: Maximum joint velocity in rad/s (default 0.5 rad/s ≈ 28.6 deg/s)
    """
    if len(target_pos) == 6:
        print("passed in:" ,target_pos)
        target_pos = np.append(target_pos,0.0)
        print("sent to:",target_pos)
    
    current_pos = robot.get_joint_pos()
    
    # Calculate the distance to travel
    distance = np.linalg.norm(target_pos - current_pos)
    
    # Calculate time based on velocity and distance
    # Use the maximum distance among all joints to determine timing
    joint_distances = np.abs(target_pos - current_pos)
    max_joint_distance = np.max(joint_distances)
    
    if max_joint_distance > 0:
        # Time = distance / velocity
        time_interval = max_joint_distance / max_velocity
        print(f"Moving {max_joint_distance:.3f} rad at {max_velocity} rad/s over {time_interval:.2f}s")
        robot.move_joints(target_pos, time_interval_s=time_interval)
    else:
        print("Already at target position")
        robot.command_joint_pos(target_pos)

def getJointPos(robot):
    return robot.get_joint_pos()
    
def moveWithKey(robot1,robot2,jointNum):
    """ A test script to move one joint, step by step according to keyboard commands"""
    leftRobot, rightRobot = robot1,robot2
    # Command the robot to move to home
    target_pos = np.array([0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    #  [wholeLeft,downEE,halfArmUP,EEUp,EELeft]
    leftRobot.command_joint_pos(target_pos)
    rightRobot.command_joint_pos(target_pos)
    while True:
        user_input = input("input = ")
        if user_input.lower() == 'a':
            target_pos[jointNum]  += .1
            print("a")
        elif user_input.lower() == 'd':
            target_pos[jointNum]  -= .1
            print("b")
        leftRobot.command_joint_pos(target_pos)
        print("sent left to", target_pos)
        time.sleep(.2)
        rightRobot.command_joint_pos(leftRobot.get_joint_pos())

def setUpRobots():
    """ returns two robots, one on each can port. 
    """
    leftRobot = get_yam_robot(channel="can1")
    righRobot = get_yam_robot(channel="can0")
    return leftRobot, righRobot
def main():
    follower_and_leader()
   
def follower_and_leader(robot1,robot2):
    """ starts the robots, one in follower mode, one in leader mode."""
    moveToPos(robot1)
    setUpInZeroG(robot1,robot2)

if __name__ == "__main__":
    main()


