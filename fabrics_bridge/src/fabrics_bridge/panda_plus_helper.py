#!/usr/bin/env python3
import numpy as np

import rospy
import rospkg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from urdf_parser_py.urdf import URDF

from fabrics_bridge.generic_helper_node import GenericHelpersNode
from fabrics_msgs.msg import (
    FabricsJointSpaceGoal,
)
from forwardkinematics.urdfFks.generic_urdf_fk import GenericURDFFk

class PandaPlusHelpersNode(GenericHelpersNode):
    def __init__(self):
        super().__init__('panda_plus_helpers_node')

    def init_robot_specifics(self):
        rospack = rospkg.RosPack()
        urdf_file = rospack.get_path('fabrics_bridge') + "/config/pandaplus.urdf"
        with open(urdf_file, 'r') as file:
            self.urdf = file.read()
        self.joint_names = [f'panda_joint{i+1}' for i in range(7)]
        self._action = np.zeros(8)
        self._planner_folder = rospack.get_path("fabrics_bridge") + "/planner/pandaplus/"
        self.urdf
        self._forward_kinematics = GenericURDFFk(
            self.urdf,
            rootLink="base_link",
            end_link=["panda_vacuum1_link" , "panda_vacuum2_link", "panda_vacuum_support_link"],
        )
        self._planner_type = 'holonomic'
        # get the joint limits for each joint
        robot = URDF.from_xml_string(self.urdf)
        self.joint_limits = [[-10, 10]]
        for i in self.joint_names:
            self.joint_limits.append(
                [
                    robot.joint_map[i].limit.lower,
                    robot.joint_map[i].limit.upper,
                ]
            )

    def joint_space_goal_cb(self, msg: FabricsJointSpaceGoal):
        self._goal_msg = msg
        self._goal_msg.goal_joint_state.position = [float(self._q[0])] + list(msg.goal_joint_state.position)

    def init_joint_states_subscriber(self):
        self._q = np.zeros(8)
        self._qdot = np.zeros(8)
        self.joint_state_subscriber = rospy.Subscriber(
            "/joint_states_filtered",
            JointState,
            self.joint_states_callback,
            tcp_nodelay=True,
        )
        self.boxer_position_subscriber = rospy.Subscriber(
            "/boxer_position",
            Float32,
            self.boxer_position_callback,
            tcp_nodelay=True,
        )

    def joint_states_callback(self, msg: JointState):
        self._q[1:] = np.array(msg.position[5:12])
        self._qdot[1:] = np.array(msg.velocity[5:12])
        self._qdot[0] = msg.velocity[3] 

    def boxer_position_callback(self, msg: Float32):
        self._q[0] = msg.data


