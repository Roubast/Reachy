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

# Take info from VR and get JointStates 
my_chain = ikpy.chain.Chain.from_urdf_file("/opt/source/ws_moveit/src/reachy_description/reachy.URDF", base_elements=["torso"], 
                                                    active_links_mask=[False, True, True, True, True, True, True, True, False])

redis_connection = redis.Redis(host='192.168.118.96', port=6379, db=0, password='DTL@b2021')
pubsub = redis_connection.pubsub()

pos_lock = threading.Lock()
cur_rot = {}
cur_pos = {}
popcop = {}
goal = np.array([])

Previous_joints_position = [0, 0, 0, 0, 0, 0, 0, 0, 0]    

import time

def peredelka(cur_pos, cur_rot, init_pos, init_rot):
    # docker run -d --name reachy_move2 -p 5922:5900 -e VNC_PASSWORD=qwe1210 -e DISPLAY=unix$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/elabuga/reachy_move/reachy_m:/opt/source reachy_f1

    global popcop, Joints_target_position, Previous_joints_position
    with pos_lock:
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
        Joints_target_position = my_chain.inverse_kinematics_frame(InvKin_Matrix, initial_position=Previous_joints_position)
        Previous_joints_position = Joints_target_position
        return Joints_target_position

def command_callback(cmd_msg):
    global cur_pos, cur_rot
    if cmd_msg is not None:
        cmd_data = json.loads(cmd_msg['data'])['data']
        if ('arm' in cmd_data) and (cmd_data['arm'] == 1):
            with pos_lock:
                cur_pos = cmd_data['pos']
                cur_rot = cmd_data['rot']

pubsub.psubscribe(**{'command': command_callback})
pubsub.run_in_thread(sleep_time=.01)

Success = False
while not Success:
    with pos_lock:
        if cur_pos:
            init_pos = {'x': cur_pos['x'] / 1000, 'y': cur_pos['y'] / 1000, 'z': cur_pos['z'] / 1000 + 1}
            init_rot = {'r': cur_rot['r'] / 180 * math.pi, 'p': cur_rot['p'] / 180 * math.pi, 'y': cur_rot['y'] / 180 * math.pi}
            Success = True

class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        #moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        #robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        #scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "right_arm"
        #move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        # display_trajectory_publisher = rospy.Publisher(
        #     "/move_group/display_planned_path",
        #     moveit_msgs.msg.DisplayTrajectory,
        #     queue_size=1,
        # )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        #planning_frame = move_group.get_planning_frame()
        #print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        #eef_link = move_group.get_end_effector_link()
        #print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        #group_names = robot.get_group_names()
        #print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        # print("============ Printing robot state")
        # print(robot.get_current_state())
        # print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        # self.box_name = ""
        # self.robot = robot
        # self.scene = scene
        # self.move_group = move_group
        # self.display_trajectory_publisher = display_trajectory_publisher
        # self.planning_frame = planning_frame
        # self.eef_link = eef_link
        # self.group_names = group_names


        display_trajectory_publisher = rospy.Publisher(
            "/right_arm_position_controller/command",
            JointTrajectory,
            queue_size=1,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        #planning_frame = move_group.get_planning_frame()
        #print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        #eef_link = move_group.get_end_effector_link()
        #print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        #group_names = robot.get_group_names()
        #print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        # print("============ Printing robot state")
        # print(robot.get_current_state())
        # print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        # self.box_name = ""
        # self.robot = robot
        # self.scene = scene
        # self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        # self.planning_frame = planning_frame
        # self.eef_link = eef_link
        # self.group_names = group_names

    def go_to_joint_state(self, goal):
        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        #joint_goal = self.move_group.get_current_joint_values()
        joint_goal = np.zeros(7)
        joint_goal[0] = goal[1]
        joint_goal[1] = goal[2]
        joint_goal[2] = goal[3]
        joint_goal[3] = goal[4]
        joint_goal[4] = goal[5]
        joint_goal[5] = goal[6]
        joint_goal[6] = goal[7]
        #joint_goal[7] = goal[8]

        #print(joint_goal)
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        #self.move_group.set_joint_value_target(joint_goal)
        #plan = self.move_group.plan()
        #start_time = time.time()
        #self.move_group.go(wait=True)
        #print(type(plan))

        # Calling ``stop()`` ensures that there is no residual movement
        #self.move_group.stop()

        #return plan

    #def execute_plan(self, plan):
        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        #self.move_group.execute(plan, wait=True)

        traj = JointTrajectory()
        traj.joint_names = ['r_shoulder_pitch' , 'r_shoulder_roll' , 'r_arm_yaw' , 'r_elbow_pitch' , 'r_forearm_yaw' , 'r_wrist_pitch' , 'r_wrist_roll']
        ptn = JointTrajectoryPoint()
        ptn.positions = joint_goal
        ptn.velocities = []
        ptn.time_from_start = rospy.Duration(1.0)
        traj.header.stamp = rospy.Time.now()
        traj.points.append(ptn)
        self.display_trajectory_publisher.publish(traj)

we = MoveGroupPythonInterfaceTutorial()

while True:
    if cur_pos:
        goal = peredelka(cur_pos, cur_rot, init_pos, init_rot)
        #print(goal)
        MoveGroupPythonInterfaceTutorial.go_to_joint_state(we, goal)
            #plan = MoveGroupPythonInterfaceTutorial.go_to_joint_state(we, goal=goal)
            #MoveGroupPythonInterfaceTutorial.execute_plan(we, plan=plan)
