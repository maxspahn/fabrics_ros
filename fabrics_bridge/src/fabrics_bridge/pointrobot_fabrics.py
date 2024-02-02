#!/usr/bin/env python3
import numpy as np
import os
import rospy
import rospkg
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

from urdf_parser_py.urdf import URDF

from fabrics_bridge.generic_fabrics_node import GenericFabricsNode
from forwardkinematics.urdfFks.generic_urdf_fk import GenericURDFFk

class PointrobotFabricsNode(GenericFabricsNode):
    def __init__(self):
        super().__init__('pointrobot_fabrics_node')

    def init_robot_specifics(self):
        self.joint_names = [f'pointrobot_joint{i+1}' for i in range(2)]
        self._action = np.zeros(3)
        rospack = rospkg.RosPack()
        self._planner_folder = rospack.get_path("fabrics_bridge") + "/planner/pointrobot/"
        absolute_path = os.path.dirname(os.path.abspath(__file__))
        with open("/home/saray/dingo_ws/src/fabrics_ros/fabrics_bridge/config/pointRobot.urdf", "r", encoding="utf-8") as file:
            self.urdf = file.read()
        self._forward_kinematics = GenericURDFFk(
            self.urdf,
            rootLink=rospy.get_param("/root_link"),
            end_link=rospy.get_param("/end_links"),
        )
        self._planner_type = rospy.get_param("/planner_type")
        # get the joint limits for each joint
        # robot = URDF.from_parameter_server(rospy.get_param("/urdf_source"))
        self.joint_limits = []
        for i in self.joint_names:
            self.joint_limits.append(
                rospy.get_param("/joint_limits"),
                    # rospy.get_param("/limits_higher")
            )

    def init_publishers(self):
        self._pointrobot_command_publisher = rospy.Publisher(
            '/cmd_vel',
            Twist, 
            queue_size=10
        )

    def init_joint_states_subscriber(self):
        #todo: remove hardcoded
        self._q = np.zeros(3)
        self._qdot = np.zeros(3)
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
        #todo: remove hardcoded part
        self._q = np.array(msg.position[0:3])
        self._qdot = np.array(msg.velocity[0:3])

    def publish_action(self):
        desired_vel= Twist()
        if np.isnan(self._action).any():
            rospy.logwarn(f"Action not a number {self._action}")
            self._action = np.zeros((rospy.get_param("/degrees_of_freedom"), 1))
            action_msg = Float64MultiArray(data=self._action)
            self._pointrobot_command_publisher.publish(action_msg)
            return
        action_msg = Float64MultiArray(data=self._action)
        desired_vel.linear.x = self._action[0]
        desired_vel.linear.y = self._action[1]
        print("publish actions!!")

        self._pointrobot_command_publisher.publish(desired_vel)
        # self._pointrobot_command_publisher.publish(action_msg)

if __name__ == "__main__":
    node = PointrobotFabricsNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass

