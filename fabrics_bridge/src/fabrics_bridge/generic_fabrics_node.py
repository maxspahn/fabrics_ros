#!/usr/bin/env python3
from abc import ABC, abstractmethod
from typing import Union
import time
import numpy as np
import rospy
import rospkg
import os
import hashlib

from std_msgs.msg import Empty
from geometry_msgs.msg import Point, Quaternion

from mpscenes.goals.goal_composition import GoalComposition

from fabrics.planner.parameterized_planner import ParameterizedFabricPlanner
from fabrics.planner.serialized_planner import SerializedFabricPlanner


from fabrics_msgs.msg import (
    FabricsJointSpaceGoal,
    FabricsPoseGoal,
    FabricsConstraintsGoal,
    FabricsObstacleArray,
)
from fabrics_bridge.goal_wrapper import FabricsGoalWrapper
from fabrics_bridge.utils import create_planner

# Helpers
def _it(self):
    yield self.x
    yield self.y
    yield self.z


Point.__iter__ = _it


def _it(self):
    yield self.x
    yield self.y
    yield self.z
    yield self.w


def list_to_unique_hash(a: list) -> str:
    string = ""
    for value in sorted(a):
        string = string.join(value)
    encoded_string = string.encode("utf-8")
    return hashlib.md5(encoded_string).hexdigest()


Quaternion.__iter__ = _it


class GenericFabricsNode(ABC):

    _planner : ParameterizedFabricPlanner
    _goal: Union[GoalComposition, None]

    def __init__(self, node_name: str = 'fabrics_node'):
        rospy.init_node(node_name)
        self.load_parameters()
        self.init_robot_specifics()
        self._goal = None
        self._goal_string = 'none'
        self._robot_type = rospy.get_param('/robot_type') 
        self._changed_planner = True
        self.stop_acc_bool = True
        self.load_all_available_planners()
        self._runtime_arguments = {}
        self.goal_wrapper = FabricsGoalWrapper()
        self._frequency = 100
        self.rate = rospy.Rate(self._frequency)
        self.init_publishers()
        self.init_subscribers()

        self.stop_acc_bool = False
        self.init_runtime_arguments()
        self.compose_runtime_obstacles_argument()
    
    @abstractmethod
    def init_robot_specifics(self):
        pass

    def load_all_available_planners(self):
        rospy.loginfo("Loading available planners.")
        self.planners = {}
        for planner_file in os.listdir(self._planner_folder):
            full_path_to_file = self._planner_folder + planner_file
            if planner_file[0] == '.':
                continue
            rospy.loginfo(f"Loading {planner_file}")
            self.planners[full_path_to_file] = SerializedFabricPlanner(full_path_to_file)
        rospy.loginfo("Loaded available planners.")
        


    def init_runtime_arguments(self):
        if self._planner_type == 'nonholonomic':
            self._runtime_arguments.update({
                "m_arm": np.array([rospy.get_param("/m_arm")]),
                "m_base_x": np.array([rospy.get_param("/m_base")]),
                "m_base_y": np.array([rospy.get_param("/m_base")]),
                "m_rot": np.array([rospy.get_param("/m_rot")])
            })
            parameters = {}
            k_fin_col = np.array([rospy.get_param("/k_fin_col")])
            k_geo_col = np.array([rospy.get_param("/k_geo_col")])
            for i in range(self.num_sphere_obstacles):
                for collision_link in self.collision_links:
                    parameters[f'k_fin_col_obst_{i}_{collision_link}_leaf'] = k_fin_col
                    parameters[f'k_geo_col_obst_{i}_{collision_link}_leaf'] = k_geo_col
            self._runtime_arguments.update(parameters)
        else:
            self._runtime_arguments = {}

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
    
    def publish_zero_velocity(self):
        self._action = np.zeros_like(self._action)
        self.publish_action()

    def load_planner(self, goal):
        self.publish_zero_velocity()

        self.stop_acc_bool = True
        if goal is None:
            return
        rospack = rospkg.RosPack()
        file_hash = self.hash_planner_configuration(goal)
        serialize_file = self._planner_folder + file_hash
        # If the planner is not serialized yet, this node has to wait for the serialization to finish.
        try:
            rospy.loginfo(f"Loading planner for goal type : {self._goal_string}")
            self._planner = self.planners[serialize_file]
            rospy.loginfo(self._planner)
            self.stop_acc_bool = False
        except Exception as e:
            rospy.logwarn(e)
            rospy.loginfo("Planner not found: Waiting...")
            self.init_planner(goal)
            self.stop_acc_bool = False
        self._changed_planner = False

    def hash_planner_configuration(self, goal: GoalComposition):
        hash_robot_type: str = rospy.get_param("/robot_type")
        hash_sphere_obstacle = hashlib.md5(str(self.num_sphere_obstacles).encode("utf-8")).hexdigest()
        hash_box_obstacle = hashlib.md5(str(self.num_box_obstacles).encode("utf-8")).hexdigest()
        hash_self_collision_pairs = list_to_unique_hash(self.self_collision_pairs)
        hash_collision_links = list_to_unique_hash(self.collision_links)
        hash_goal = self.goal_wrapper.hash_goal(goal)
        composed_hash = (
            hash_robot_type
            + hash_sphere_obstacle
            + hash_box_obstacle
            + hash_self_collision_pairs
            + hash_collision_links
            + hash_goal
        )
        short_hash = hashlib.md5(composed_hash.encode('utf-8')).hexdigest()
        return short_hash

    def init_planner(self, goal: GoalComposition):
        file_hash = self.hash_planner_configuration(goal)
        serialize_file = self._planner_folder + file_hash
        if os.path.exists(serialize_file):
            rospy.loginfo(f"Planner is already serialized in {serialize_file}")
            self.planners[serialize_file] = SerializedFabricPlanner(serialize_file)
            return


        goal_string = self.goal_wrapper.goal_string(goal)
        rospy.loginfo(f"Creating the planner for goal:  {goal_string}")
        t0 = time.perf_counter()
        planner = create_planner(
            self._planner_type,
            self._forward_kinematics
        )
        # The planner hides all the logic behind the function set_components.
        #dummy_goal = self.goal_wrapper.compose_dummy_goal(goal_type)
        planner.set_components(
            collision_links=self.collision_links,
            self_collision_pairs=self.self_collision_pairs,
            goal=goal,
            limits=self.joint_limits,
            number_obstacles=self.num_sphere_obstacles,
            number_obstacles_cuboid=self.num_box_obstacles,
        )
        planner.concretize(mode='vel', time_step = 10/self._frequency)
        t1 = time.perf_counter()
        composition_time = (t1-t0) * 1000
        # serializing the planner
        rospy.loginfo(f"Planner composed in {composition_time} ms")
        goal_string = self.goal_wrapper.goal_string(goal)
        rospy.loginfo(f"Serializing the planner for goal:  {goal_string}")
        t2 = time.perf_counter()
        planner.serialize(serialize_file)
        t3 = time.perf_counter()
        serialize_time = (t3-t2) * 1000
        rospy.loginfo(f"Finished serializing, {serialize_file} has been created in {serialize_time} ms")
        self.planners[serialize_file] = planner


    def run(self):
        while not rospy.is_shutdown():
            # act and evaluate only when a goal, joint_states and a correct planner are received
            t0 = time.perf_counter()
            if (
                hasattr(self, '_goal')
                and not self.stop_acc_bool
                and not self._goal is None
            ):
                if self._changed_planner:
                    self.load_planner(self._goal)
                self.act()
            t1 = time.perf_counter()
            #rospy.loginfo(f"Compute time in ms : {(t1-t0) * 1000}")
            self.rate.sleep()

    @abstractmethod
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
        # preempt the previous goal once this cb is triggered
        self.preempt_goal_subscriber = rospy.Subscriber(
            "preempt_goal", Empty, self.preempt_goal_callback
        )
        self.init_joint_states_subscriber()

    def obs_callback(self, msg: FabricsObstacleArray):
        self.obstacles = msg.obstacles
        self._obstacle_runtime_arguments = self.compose_runtime_obstacles_argument()

    def joint_space_goal_cb(self, msg: FabricsJointSpaceGoal):
        self._goal_msg = msg
        self._goal = self.goal_wrapper.wrap(msg)
        goal_string = self.goal_wrapper.goal_string(self._goal)
        if self._goal_string != goal_string:
            self._changed_planner = True
        self._goal_string = goal_string

    def ee_pose_goal_cb(self, msg: FabricsPoseGoal):
        self._goal_msg = msg
        self._goal = self.goal_wrapper.wrap(msg)
        goal_string = self.goal_wrapper.goal_string(self._goal)
        if self._goal_string != goal_string:
            self._changed_planner = True
        self._goal_string = goal_string

    def constraints_goal_cb(self, msg: FabricsConstraintsGoal):
        self._goal_msg = msg
        self._goal = self.goal_wrapper.wrap(msg)
        goal_string = self.goal_wrapper.goal_string(self._goal)
        if self._goal_string != goal_string:
            self._changed_planner = True
        self._goal_string = goal_string

    @abstractmethod
    def init_joint_states_subscriber(self):
        pass

    def preempt_goal_callback(self, msg: Empty):
        rospy.loginfo(f"goal preempted")
        # only change the weights
        self._goal_msg = FabricsJointSpaceGoal()
        self._goal_msg.goal_joint_state.position = self._q.tolist()
        self._goal_msg.weight=0
        self._goal = self.goal_wrapper.wrap(self._goal_msg)

    def compose_runtime_obstacles_argument(self):
        x_obst_sphere = np.full((self.num_sphere_obstacles, 3), [100.0] * 3)
        radius_obst = np.full((self.num_sphere_obstacles, 1), 0.05)
        x_obsts_cuboid = np.full((self.num_box_obstacles, 3), [100.0] * 3)
        size_obsts_cuboid = np.full((self.num_box_obstacles, 3), 0.05)

        for i, o in enumerate(self.obstacles):
            if o.obstacle_type == "sphere":
                if i >= self.num_sphere_obstacles:
                    rospy.logwarn(
                        f"Fabrics planner received more sphere obstacles than it can process: can only handle {self.num_sphere_obstacles}. Consider increasing the rosparameter num_sphere_obstacles."
                    )
                    break
                x_obst_sphere[i, :] = list(o.position)
                radius_obst[i] = o.radius
            elif o.obstacle_type == "box":
                if i >= self.num_box_obstacles:
                    rospy.logwarn(
                        f"Fabrics planner received more box obstacles than it can process: can only handle {self.num_box_obstacles}. Consider increasing the rosparameter num_box_obstacles."
                    )
                    break
                x_obsts_cuboid[i, :] = list(o.position)
                if i == 0:
                    size_obsts_cuboid[0] = o.size.x
                elif i == 1:
                    size_obsts_cuboid[1] = o.size.y
                else: 
                    size_obsts_cuboid[2] = o.size.z
        self._runtime_arguments.update(self.collision_bodies)
        self._runtime_arguments.update({
            'x_obst': x_obst_sphere, 
            'radius_obst': radius_obst,
            'x_obsts_cuboid': x_obsts_cuboid,
            'size_obsts_cuboid': size_obsts_cuboid,
        })


    @abstractmethod
    def set_joint_states_values(self):
        pass

    def compute_action(self):
        self.goal_wrapper.compose_runtime_arguments(self._goal, self._runtime_arguments)
        self.set_joint_states_values()
        action = self._planner.compute_action(
            **self._runtime_arguments,
        )
        return action

    @abstractmethod
    def publish_action(self):
        """
        self._vel_msg_panda.data = self._action.tolist()[2:]
        self._vel_msg_boxer.linear.x = self._action[0]
        self._vel_msg_boxer.angular.z = self._action[1]
        self._vel_pub_panda.publish(self._vel_msg_panda)
        if self._robot_type in ["albert", "boxer"]:
            self._vel_pub_boxer.publish(self._vel_msg_boxer)
        """
        pass

    def act(self):
        alpha = 0.95
        try:
            action = np.zeros_like(self._action)
            action = self.compute_action()
            self._action = self._action * alpha + action * (1-alpha)
            self._action = np.clip(self._action, self._min_vel, self._max_vel)
            self.publish_action()
        except Exception as e:
            rospy.loginfo(f"Not planning due to error {e}")
            rospy.loginfo("Waiting for correct planner to be loaded.")
            self.load_planner(self._goal)
