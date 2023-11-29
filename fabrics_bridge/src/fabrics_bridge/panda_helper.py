#!/usr/bin/env python3
import numpy as np

import rospy
from sensor_msgs.msg import JointState

from urdf_parser_py.urdf import URDF

from fabrics_bridge.generic_helper_node import GenericHelpersNode
from forwardkinematics.urdfFks.generic_urdf_fk import GenericURDFFk

class PandaHelpersNode(GenericHelpersNode):
    def __init__(self):
        super().__init__('panda_helpers__node')

    def init_robot_specifics(self):
        self.joint_names = [f'panda_joint{i+1}' for i in range(7)]
        self._action = np.zeros(7)
        self._forward_kinematics = GenericURDFFk(
            self.urdf,
            rootLink=rospy.get_param("/root_link"),
            end_link=rospy.get_param("/end_links"),
        )
        self._planner_type = 'holonomic'
        # get the joint limits for each joint
        robot = URDF.from_parameter_server(rospy.get_param("/urdf_source"))
        self.joint_limits = []
        for i in self.joint_names:
            self.joint_limits.append(
                [
                    robot.joint_map[i].limit.lower,
                    robot.joint_map[i].limit.upper,
                ]
            )

    def init_joint_states_subscriber(self):
        self._q = np.zeros(7)
        self._qdot = np.zeros(7)
        self.joint_state_subscriber = rospy.Subscriber(
            "/joint_states_filtered",
            JointState,
            self.joint_states_callback,
            tcp_nodelay=True,
        )

    def set_joint_states_values(self):
        self._runtime_arguments['q'] = self._q
        self._runtime_arguments['qdot'] = self._qdot

    def joint_states_callback(self, msg: JointState):
        self._q = np.array(msg.position[5:12])
        self._qdot = np.array(msg.velocity[5:12])


