#!/usr/bin/env python3
from abc import ABC, abstractmethod
import numpy as np
import rospy

from std_msgs.msg import Empty


from fabrics_msgs.msg import (
    FabricsJointSpaceGoal,
    FabricsPoseGoal,
    FabricsConstraintsGoal,
    FabricsObstacleArray,
)
from fabrics_bridge.marker_manager import FabricsMarkerManager
from fabrics_bridge.goal_evaluator import FabricsGoalEvaluator

class GenericHelpersNode(ABC):
    def __init__(self, node_name: str = "fabrics_helpers"):
        self._node_name = node_name
        rospy.init_node(self._node_name)
        self._goal_msg = None
        self.obstacles = []
        self._rate = rospy.Rate(5)
        self.load_parameters()
        self.init_robot_specifics()
        self.init_publishers()
        self.init_subscribers()
        self.state_evaluator = FabricsGoalEvaluator(self._forward_kinematics)
        self.marker_manager = FabricsMarkerManager(self.num_sphere_obstacles, self.collision_bodies, self.collision_links, self.self_collision_pairs)

        self.stop_acc_bool = False
        self.init_joint_states_subscriber()

    @abstractmethod
    def init_joint_states_subscriber(self):
        pass

    @abstractmethod
    def init_robot_specifics(self):
        pass

    def init_publishers(self):
        pass

    def init_subscribers(self):
        self.obs_subscriber = rospy.Subscriber(
            "planning_obs", FabricsObstacleArray, self.obs_callback,
            tcp_nodelay=True,
        )
        self.obstacles = []
        rospy.Subscriber(
            "joint_space_goal", FabricsJointSpaceGoal, self.joint_space_goal_cb,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            "ee_pose_goal", FabricsPoseGoal, self.ee_pose_goal_cb,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            "constraints_goal", FabricsConstraintsGoal, self.constraints_goal_cb,
            tcp_nodelay=True,
        )
        self.init_joint_states_subscriber()

    def load_parameters(self):
        scaling_factor = rospy.get_param('/velocity_scaling')
        self._max_vel = np.array(rospy.get_param("/velocity_limits/upper")) * scaling_factor
        self._min_vel = np.array(rospy.get_param("/velocity_limits/lower")) * scaling_factor
        self.num_sphere_obstacles = rospy.get_param("/num_sphere_obs")
        self.num_box_obstacles = rospy.get_param("/num_box_obs")
        # robot_description should be global because it's not part of fabrics ns
        try:
            self.urdf = rospy.get_param(rospy.get_param("/urdf_source"))
        except KeyError:
            rospy.logwarn(f"Fabrics specific urdf not found.")
            self.urdf = None
        self.collision_links = rospy.get_param("/collision_links")
        try:
            self.self_collision_pairs = rospy.get_param("/self_collision_pairs")
            if self.self_collision_pairs == "None":
                self.self_collision_pairs = {}
        except KeyError as e:
            self.self_collision_pairs = {}
        self.collision_bodies = rospy.get_param("/collision_bodies")
        # TODO: Currently, fabrics require inputs to be np.arrays. This should be chaged to also allow floats.
        for name, radius in self.collision_bodies.items():
            self.collision_bodies[name] = np.array([radius])

    def obs_callback(self, msg: FabricsObstacleArray):
        self.obstacles = msg.obstacles

    def joint_space_goal_cb(self, msg: FabricsJointSpaceGoal):
        self._goal_msg = msg

    def ee_pose_goal_cb(self, msg: FabricsPoseGoal):
        self._goal_msg = msg

    def constraints_goal_cb(self, msg: FabricsConstraintsGoal):
        self._goal_msg = msg

    def run(self):
        while not rospy.is_shutdown():
            if self._goal_msg:
                goal_is_reached = self.state_evaluator.evaluate(self._goal_msg, self._q)
                self.marker_manager.update_goal_markers(self._goal_msg, goal_is_reached)
                self.marker_manager.update_obstacle_markers(self.obstacles)
            self._rate.sleep()
