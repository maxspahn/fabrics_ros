#!/usr/bin/env python3
from geometry_msgs.msg import Twist
import numpy as np

import rospy
import rospkg
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from fabrics_bridge.generic_fabrics_node import GenericFabricsNode
from forwardkinematics.urdfFks.generic_urdf_fk import GenericURDFFk


class BoxerFabricsNode(GenericFabricsNode):
    def __init__(self):
        super().__init__('boxer_fabrics_node')

    def init_robot_specifics(self):
        self._action = np.zeros(2)
        rospack = rospkg.RosPack()
        urdf_file = rospack.get_path('fabrics_bridge') + "/config/boxer.urdf"
        with open(urdf_file, 'r') as file:
            self.urdf = file.read()
        self._planner_folder = rospack.get_path("fabrics_bridge") + "/planner/boxer/"
        self._forward_kinematics = GenericURDFFk(
            self.urdf,
            rootLink='base_link',
            end_link='ee_link',
            base_type='diffdrive',
        )
        self._planner_type = 'nonholonomic'
        self.joint_limits = [[-10, 10], [-10, 10], [-10, 10]]

    def init_publishers(self):
        self._boxer_command_publisher = rospy.Publisher(
            '/cmd_vel',
            Twist,
            queue_size=10
        )

    def set_joint_states_values(self):
        self._runtime_arguments['q'] = self._q
        self._runtime_arguments['qdot'] = self._qdot
        self._runtime_arguments['qudot'] = self._qudot

    def init_joint_states_subscriber(self):
        self._q = np.zeros(3)
        self._qdot = np.zeros(3)
        self._qudot = np.zeros(2)
        self.joint_state_subscriber = rospy.Subscriber(
            "/joint_states_filtered",
            JointState,
            self.joint_states_callback,
            tcp_nodelay=True,
        )

    def joint_states_callback(self, msg: JointState):
        self._q = np.array(msg.position[0:3])
        self._qdot = np.array(msg.velocity[0:3])
        self._qudot = np.array(msg.velocity[3:5])

    def publish_action(self):
        action_msg  = Twist()
        action_msg.linear.x = self._action[0]
        action_msg.angular.z = self._action[1]

        self._boxer_command_publisher.publish(action_msg)

if __name__ == "__main__":
    node = PandaFabricsNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass

