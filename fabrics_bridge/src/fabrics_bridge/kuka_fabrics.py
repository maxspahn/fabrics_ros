#!/usr/bin/env python3
import numpy as np
import os
import rospy
import rospkg
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from fabrics_msgs.msg import FabricsObstacleArray, FabricsObstacle
from cor_tud_msgs.msg import ControlRequest

from urdf_parser_py.urdf import URDF

from fabrics_bridge.generic_fabrics_node import GenericFabricsNode
from forwardkinematics.urdfFks.generic_urdf_fk import GenericURDFFk

class KukaFabricsNode(GenericFabricsNode):
    def __init__(self):
        super().__init__('kuka_fabrics_node')

    def init_robot_specifics(self):
        self.dof = rospy.get_param("/degrees_of_freedom")
        self.joint_names = [f'pointrobot_joint{i+1}' for i in range(self.dof)]
        self._action = np.zeros(self.dof)
        rospack = rospkg.RosPack()
        self._planner_folder = rospack.get_path("fabrics_bridge") + "/planner/kuka/"
        absolute_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
        print("absolute path: ", absolute_path)
        with open(absolute_path+"/config/iiwa7.urdf", "r", encoding="utf-8") as file:
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
            )
        self.obstacle1_msg = None

    def init_publishers(self):
        self._kuka_command_publisher = rospy.Publisher(
            '/%s/control_request' % "iiwa7",
            ControlRequest, 
            queue_size=10
        )

        self._obstacle_planner_publisher = rospy.Publisher(
            '/fabrics/planning_obs',
            FabricsObstacleArray, 
            queue_size=10
        )

    def init_joint_states_subscriber(self):
        self._q = np.zeros(self.dof)
        self._qdot = np.zeros(self.dof)
        self.joint_state_subscriber = rospy.Subscriber(
            "/%s/joint_states" % "iiwa7",
            JointState,
            self.joint_states_callback,
            tcp_nodelay=True,
        )

    def init_obstacle_subscriber(self):
        self.obstacle_subscriber = rospy.Subscriber('vicon/obstacle1', PoseWithCovarianceStamped, self.cb_obstacle1, tcp_nodelay=True)

    def cb_obstacle1(self, msg):
        # This function is called whenever a message is received on the subscribed topic
        self.obstacle1_msg = msg
        self.obstacle1_received == True

    def set_joint_states_values(self):
        self._runtime_arguments['q'] = self._q
        self._runtime_arguments['qdot'] = self._qdot

    def joint_states_callback(self, msg: JointState):
        self._q = np.array(msg.position[0:self.dof])
        self._qdot = np.array(msg.velocity[0:self.dof])

    def send_request(self, q_d, q_dot_d=[0, 0, 0, 0, 0, 0, 0]):
        msg = ControlRequest()
        msg.header.stamp = rospy.Time.now()
        msg.q_d.data = q_d
        msg.q_dot_d.data = q_dot_d
        msg.control_type = 'joint impedance'
        self._kuka_command_publisher.publish(msg)
        return True
    
    def check_dist_to_goal(self, q_goal):
        np.linalg.norm(self._q, q_goal)
        
    
    def publish_action(self):
        if np.isnan(self._action).any():
            rospy.logwarn(f"Action not a number {self._action}")
            self._action = np.zeros((rospy.get_param("/degrees_of_freedom"), 1))
            self.send_request(q_d = self._q, q_dot_d = self._action)
            return
        self.send_request(q_d = self._q, q_dot_d = self._action)
        return
        

if __name__ == "__main__":
    node = KukaFabricsNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass

