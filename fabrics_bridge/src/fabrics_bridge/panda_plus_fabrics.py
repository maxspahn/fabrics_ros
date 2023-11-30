import numpy as np
from std_msgs.msg import Empty

import rospy
import rospkg
from std_msgs.msg import Float32, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

from urdf_parser_py.urdf import URDF

from fabrics_bridge.generic_fabrics_node import GenericFabricsNode
from fabrics_msgs.msg import (
    FabricsJointSpaceGoal,
    FabricsPoseGoal,
    FabricsConstraintsGoal,
)
from forwardkinematics.urdfFks.generic_urdf_fk import GenericURDFFk

class PandaPlusFabricsNode(GenericFabricsNode):
    _publish_boxer: bool

    def __init__(self):
        self._publish_boxer = True
        super().__init__('panda_plus_fabrics_node')

    def load_planner(self, goal):
        self.reset_boxer_position()
        return super().load_planner(goal)

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

    def ee_pose_goal_cb(self, msg: FabricsPoseGoal):
        super().ee_pose_goal_cb(msg)
        self._publish_boxer = True

    def constraints_goal_cb(self, msg: FabricsConstraintsGoal):
        super().constraints_goal_cb(msg)
        self._publish_boxer = True


    def joint_space_goal_cb(self, msg: FabricsJointSpaceGoal):
        self._goal_msg = msg
        self._goal_msg.goal_joint_state.position = [float(self._q[0])] + list(msg.goal_joint_state.position)
        self._goal = self.goal_wrapper.wrap(self._goal_msg)
        goal_string = self.goal_wrapper.goal_string(self._goal)
        if self._goal_string != goal_string:
            self._changed_planner = True
        self._goal_string = goal_string
        self._publish_boxer = False

    def reset_boxer_position(self):
        self._boxer_reset_publisher.publish(Empty())

    def init_publishers(self):
        self._panda_command_publisher = rospy.Publisher(
            f'/panda_joint_velocity_controller/command',
            Float64MultiArray,
            queue_size=10
        )
        self._boxer_command_publisher = rospy.Publisher(
            f'/cmd_vel',
            Twist,
            queue_size=10
        )
        self._boxer_reset_publisher = rospy.Publisher(
            '/boxer_position/reset',
            Empty,
            queue_size=1,
        )

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

    def set_joint_states_values(self):
        self._runtime_arguments['q'] = self._q
        self._runtime_arguments['qdot'] = self._qdot

    def joint_states_callback(self, msg: JointState):
        self._q[1:] = np.array(msg.position[5:12])
        self._qdot[1:] = np.array(msg.velocity[5:12])
        self._qdot[0] = msg.velocity[3] 

    def boxer_position_callback(self, msg: Float32):
        self._q[0] = msg.data

    def publish_action(self):
        if np.isnan(self._action).any():
            rospy.logwarn(f"Action not a number {self._action}")
            return
        action_msg = Float64MultiArray(data=self._action[1:])
        boxer_msg = Twist()
        boxer_msg.linear.x = 0.3 * self._action[0]
        self._panda_command_publisher.publish(action_msg)
        if self._publish_boxer:
            self._boxer_command_publisher.publish(boxer_msg)
        

if __name__ == "__main__":
    node = PandaFabricsNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass

