#!/usr/bin/python3
# Libraries for RVIS 
# from __future__ import print_function
# from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list

# Libraries for VR + IK
import json
import math
import redis
import threading
from scipy.spatial.transform import Rotation
import numpy as np
import ikpy.chain

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import time

class RightArmControl(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(RightArmControl, self).__init__()
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
        group_name = "right_arm"
        display_trajectory_publisher = rospy.Publisher(
            "/reachy/right_arm_position_controller/command",
            JointTrajectory,
            queue_size=1,
        )
        self.display_trajectory_publisher = display_trajectory_publisher
        self.Previous_joints_position = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    def go_to_joint_state(self, goal):
        joint_goal = np.zeros(7)
        joint_goal[0] = goal[1]
        joint_goal[1] = goal[2]
        joint_goal[2] = goal[3]
        joint_goal[3] = goal[4]
        joint_goal[4] = goal[5]
        joint_goal[5] = goal[6]
        joint_goal[6] = goal[7]

        traj = JointTrajectory()
        traj.joint_names = ['r_shoulder_pitch' , 'r_shoulder_roll' , 'r_arm_yaw' , 'r_elbow_pitch' , 'r_forearm_yaw' , 'r_wrist_pitch' , 'r_wrist_roll']
        ptn = JointTrajectoryPoint()
        ptn.positions = joint_goal
        ptn.velocities = []
        ptn.time_from_start = rospy.Duration(1.0)
        traj.header.stamp = rospy.Time.now()
        traj.points.append(ptn)
        self.display_trajectory_publisher.publish(traj)

    def peredelka(self, cur_pos, cur_rot, init_pos, init_rot):
        start_time = time.time()
        rot = Rotation.from_euler('xyz', [cur_rot['r'] / 180 * math.pi - init_rot['r'], 
                                    cur_rot['p'] / 180 * math.pi - init_rot['p'],
                                    cur_rot['y'] / 180 * math.pi - init_rot['y']], degrees = False).as_matrix()
        popcop['y'] = cur_pos['x'] / 1000 - init_pos['x'] - 0.19
        popcop['x'] = cur_pos['y'] / 1000 - init_pos['y']
        popcop['z'] = cur_pos['z'] / 1000 + 1 - init_pos['z'] - 0.5625 #+ 0.4375
        pos = np.array([[popcop['x']], [popcop['y']], [popcop['z']]])
        fourth_string = np.array([[0, 0, 0, 1]])
        InvKin_Matrix = np.concatenate((np.concatenate((rot, pos), axis=1), fourth_string), axis=0)
        Joints_target_position = OurChain.inverse_kinematics_frame(InvKin_Matrix, initial_position=self.Previous_joints_position)
        self.Previous_joints_position = Joints_target_position
        return Joints_target_position

class HeadControl(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(HeadControl, self).__init__()
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
        group_name = "head"
        display_trajectory_publisher = rospy.Publisher(
            "/reachy/head_controller/command",
            JointTrajectory,
            queue_size=1,
        )
        self.display_trajectory_publisher = display_trajectory_publisher
        self.Previous_joints_position = [0, 0, 0, 0, 0]

    def go_to_joint_state(self, cur_rot, head_init_rot, cur_pos, head_init_pos):
        joint_goal = np.zeros(3)
        joint_goal[0] = -cur_rot['r'] / 180 * math.pi + head_init_rot['r'] #naklon golovi vpered nazad == middle2top
        joint_goal[1] = -cur_rot['p'] / 180 * math.pi + head_init_rot['p'] #povorot golovi v storoni == bottom2middle
        joint_goal[2] = cur_rot['y'] / 180 * math.pi - head_init_rot['y'] #naklon golovi k plecham == top2head
        while joint_goal[0] > math.pi:
            joint_goal[0] -= math.pi * 2
        while joint_goal[0] < -math.pi:
            joint_goal[0] += math.pi * 2
        while joint_goal[1] > math.pi:
            joint_goal[1] -= math.pi * 2
        while joint_goal[1] < -math.pi:
            joint_goal[1] += math.pi * 2
        while joint_goal[2] < -math.pi:
            joint_goal[2] += math.pi * 2
        while joint_goal[2] > math.pi:
            joint_goal[1] -= math.pi * 2

        #print(joint_goal, cur_rot)

        traj = JointTrajectory()
        traj.joint_names = ['top2head', 'middle2top', 'bottom2middle']
        ptn = JointTrajectoryPoint()
        ptn.positions = joint_goal
        ptn.velocities = []
        ptn.time_from_start = rospy.Duration(1.0)
        traj.header.stamp = rospy.Time.now()
        traj.points.append(ptn)
        self.display_trajectory_publisher.publish(traj)

def command_callback_arm(cmd_msg):
    global cur_pos_arm, cur_rot_arm, cmd_data_arm, cmd_cmd
    if cmd_msg is not None:
        with pos_lock:
            cmd_data_arm = json.loads(cmd_msg['data'])['data']
            if ('arm' in cmd_data_arm) and (cmd_data_arm['arm'] == 1):
                cur_pos_arm = cmd_data_arm['pos']
                cur_rot_arm = cmd_data_arm['rot']

def command_callback_head(cmd_msg):
    global cur_pos_head, cur_rot_head, cmd_data_head, cmd_cmd
    if cmd_msg is not None:
        with pos_lock:
            cmd_cmd = json.loads(cmd_msg['data'])['command']
            cmd_data_head = json.loads(cmd_msg['data'])['data']
            if cmd_cmd == 'HEAD':
                cur_pos_head = cmd_data_head['pos']
                cur_rot_head = cmd_data_head['rot']

OurChain = ikpy.chain.Chain.from_urdf_file("/opt/source/ws_moveit/src/reachy_moveit_config/urdf/reachy.URDF", base_elements=["torso"], 
                                                    active_links_mask=[False, True, True, True, True, True, True, True, False])

HeadChain = ikpy.chain.Chain.from_urdf_file("/opt/source/ws_moveit/src/reachy_moveit_config/urdf/reachy.URDF", base_elements=['bottom_orbita_arm'], 
                                                    active_links_mask=[False, True, True, True, False])

redis_connection = redis.Redis(host='192.168.118.16', port=6379, db=0, password='DTL@b2021')

cur_rot_arm = {}
cur_pos_arm = {}
cur_rot_head = {}
cur_pos_head = {}
cmd_data_arm = {}
cmd_cmd = ''
cmd_data_head = {}

pubsub_arm = redis_connection.pubsub()
pos_lock = threading.Lock()
pubsub_arm.psubscribe(**{'command': command_callback_arm})
pubsub_arm.run_in_thread(sleep_time=.01)

pubsub_head = redis_connection.pubsub()
pubsub_head.psubscribe(**{'command': command_callback_head})
pubsub_head.run_in_thread(sleep_time=.01)

popcop = {}
goal = np.array([])

RobotArm = RightArmControl()
RobotHead = HeadControl()

TimeForArm = True

ArmSuccess = False
while not ArmSuccess:
    with pos_lock:
        if ('arm' in cmd_data_arm) and (cmd_data_arm['arm'] == 1):
            if cur_pos_arm:
                arm_init_pos = {'x': cur_pos_arm['x'] / 1000, 'y': cur_pos_arm['y'] / 1000, 'z': cur_pos_arm['z'] / 1000 + 1}
                arm_init_rot = {'r': cur_rot_arm['r'] / 180 * math.pi, 'p': cur_rot_arm['p'] / 180 * math.pi, 'y': cur_rot_arm['y'] / 180 * math.pi}
                ArmSuccess = True

HeadSuccess = False
while not HeadSuccess:
    with pos_lock:
        if cmd_cmd == 'HEAD':
            if cur_pos_head:
                head_init_pos = {'x': cur_pos_head['x'] / 1000, 'y': cur_pos_head['y'] / 1000, 'z': cur_pos_head['z'] / 1000 + 1}
                head_init_rot = {'r': cur_rot_head['r'] / 180 * math.pi, 'p': cur_rot_head['p'] / 180 * math.pi, 'y': cur_rot_head['y'] / 180 * math.pi}
                HeadSuccess = True
MoveArm = True
while True:
    time.sleep(0.01)
    with pos_lock:
        time1 = time.time()
        goal = RightArmControl.peredelka(RobotArm, cur_pos_arm, cur_rot_arm, arm_init_pos, arm_init_rot)
        RightArmControl.go_to_joint_state(RobotArm, goal)
        HeadControl.go_to_joint_state(RobotHead, cur_rot_head, head_init_rot, cur_pos_head, head_init_pos)
            