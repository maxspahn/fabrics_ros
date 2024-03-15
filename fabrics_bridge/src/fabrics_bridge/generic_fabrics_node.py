#!/usr/bin/env python3
from abc import ABC, abstractmethod
from typing import Union
import yaml
import time
import numpy as np
import rospy
import rospkg
import os
import hashlib
import copy

from std_msgs.msg import Empty
from geometry_msgs.msg import Point, Quaternion, PoseStamped

from mpscenes.goals.goal_composition import GoalComposition
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from fabrics.planner.parameterized_planner import ParameterizedFabricPlanner
from fabrics.planner.serialized_planner import SerializedFabricPlanner


from fabrics_msgs.msg import (
    FabricsJointSpaceGoal,
    FabricsPoseGoal,
    FabricsConstraintsGoal,
    FabricsObstacleArray,
    FabricsObstacle,
)
from fabrics_bridge.goal_wrapper import FabricsGoalWrapper
from fabrics_bridge.utils import create_planner, list_to_unique_hash
from fabrics_bridge.utils import Point

class GenericFabricsNode(ABC):

    _planner : ParameterizedFabricPlanner
    _goal: Union[GoalComposition, None]

    def __init__(self, node_name: str = 'fabrics_node'):
        rospy.init_node(node_name)
        print("init!!")
        self.load_parameters()
        self.init_robot_specifics()
        self._goal = None
        self._goal_string = 'none'
        self._frequency = 100
        self.rate = rospy.Rate(self._frequency)
        self._robot_type = rospy.get_param('/robot_type') 
        self._dim_state = rospy.get_param('/dim_state')
        self.num_plane_constraints = rospy.get_param('/num_plane_constraints')
        self.root_link = rospy.get_param('/root_link')
        self.end_link = rospy.get_param('/end_effector_link')
        self._changed_planner = True
        self.stop_acc_bool = True
        self.goal_wrapper = FabricsGoalWrapper()
        # self.load_all_available_planners()
        self._runtime_arguments = {}
        self.init_publishers()
        self.init_subscribers()
        self.planners = {}
        self.obstacles = []
        self.obstacle1_received = False
        self.obstacle1_msg = None
        self.stop_acc_bool = False
        self.goal_reached = False
        self.init_runtime_arguments()
        self.compose_runtime_obstacles_argument()
        print("end init!!")
    
    @abstractmethod
    def init_robot_specifics(self):
        pass

    def load_all_available_planners(self):
        rospy.loginfo("Loading available planners.")
        self.planners = {}
        for planner_file in os.listdir(self._planner_folder):
            full_path_to_file = self._planner_folder + planner_file
            if planner_file[0] == '.' or 'all' in planner_file:
                continue
            rospy.loginfo(f"Loading {planner_file}")
            self.planners[full_path_to_file] = SerializedFabricPlanner(full_path_to_file)
        rospy.loginfo("Loaded available planners.")
        rospy.loginfo("Generate planners from goal list.")
        self.generate_all_planners()
        rospy.loginfo("Generated all planners from goal list.")

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
        elif self._planner_type == "holonomic":
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
        if serialize_file in self.planners:
            rospy.loginfo(f"Loading planner for goal type : {self._goal_string}")
            self._planner = self.planners[serialize_file]
            self.stop_acc_bool = False
        else:
            rospy.loginfo("Creating the planner for {self._goal_string}")
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

    def generate_all_planners(self):
        goal_summary_file = self._planner_folder + "all_goals.yaml"
        with open(goal_summary_file, "r") as f:
            all_goals = yaml.safe_load(f)
        for file_hash, goal_dict in all_goals.items():
            serialize_file = self._planner_folder + file_hash
            if serialize_file not in self.planners:
                rospy.loginfo(f"Regenerating planner with hash {file_hash}")
                goal = GoalComposition(name="goal1", content_dict=goal_dict)
                self.init_planner(goal)


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
        print("self._planner_type: ", self._planner_type)
        print("self._forward_kinematics: ", self._forward_kinematics)
        planner = create_planner(
            self._planner_type,
            self._forward_kinematics
        )
        # The planner hides all the logic behind the function set_components.
        planner.set_components(
            # collision_links=self.collision_links,
            # self_collision_pairs=self.self_collision_pairs,
            goal=goal,
            # limits=self.joint_limits,
            # number_obstacles=self.num_sphere_obstacles,
            # number_obstacles_cuboid=self.num_box_obstacles,
            # number_plane_constraints=self.num_plane_constraints
        )
        planner.concretize(mode='vel', time_step = 1/self._frequency)
        t1 = time.perf_counter()
        composition_time = (t1-t0) * 1
        # serializing the planner
        rospy.loginfo(f"Planner composed in {composition_time} s")
        goal_string = self.goal_wrapper.goal_string(goal)
        rospy.loginfo(f"Serializing the planner for goal:  {goal_string}")
        t2 = time.perf_counter()
        planner.serialize(serialize_file)
        t3 = time.perf_counter()
        serialize_time = (t3-t2) * 1
        rospy.loginfo(f"Finished serializing, {serialize_file} has been created in {serialize_time} s")
        self.planners[serialize_file] = planner
        goal_summary_file = self._planner_folder + "all_goals.yaml"
        f = open(goal_summary_file, "a")
        yaml.dump({file_hash: goal.dict()}, f)
        f.close()


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
            # rospy.spin()

    @abstractmethod
    def init_publishers(self):
        pass

    def publish_obstacles(self):
        obstacle_struct = FabricsObstacle()
        obstacle_struct.radius = rospy.get_param("/radius_obst")
        obstacle_struct.obstacle_type = rospy.get_param("/obstacle_type")
        obstacles_struct = FabricsObstacleArray()
        # print("obstacle1_message:", self.obstacle1_msg)
        if self.obstacle1_msg is not None:
            obstacle_struct.position = self.obstacle1_msg.pose.pose.position
            obstacles_struct.obstacles = [obstacle_struct]
            self._obstacle_planner_publisher.publish(obstacles_struct)
        else:
            xyz_obst = rospy.get_param("/xyz_obstacle")
            obstacle_struct.position.x = xyz_obst[0]
            obstacle_struct.position.y = xyz_obst[1]
            if self._dim_state > 2:
                obstacle_struct.position.z = xyz_obst[2]
            obstacles_struct.obstacles = [obstacle_struct]
            self._obstacle_planner_publisher.publish(obstacles_struct)

    def init_subscribers(self):
        self.obs_subscriber = rospy.Subscriber(
            "fabrics/planning_obs", FabricsObstacleArray, self.obs_callback,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            "fabrics/joint_space_goal", FabricsJointSpaceGoal, self.joint_space_goal_cb,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            "fabrics/ee_pose_goal", FabricsPoseGoal, self.ee_pose_goal_cb,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            "fabrics/constraints_goal", FabricsConstraintsGoal, self.constraints_goal_cb,
            tcp_nodelay=True,
        )
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.cb_goal_rviz,
                         tcp_nodelay=True,)
        # preempt the previous goal once this cb is triggered
        self.preempt_goal_subscriber = rospy.Subscriber(
            "preempt_goal", Empty, self.preempt_goal_callback
        )

        # self.obstacle_subscriber = rospy.Subscriber('vicon/obstacle1', PoseWithCovarianceStamped, self.cb_obstacle1, tcp_nodelay=True)
        self.init_obstacle_subscriber()
        self.init_joint_states_subscriber()

    @abstractmethod
    def init_obstacle_subscriber(self):
        pass

    def obs_callback(self, msg: FabricsObstacleArray):
        self.obstacles = msg.obstacles
        self._obstacle_runtime_arguments = self.compose_runtime_obstacles_argument()

    def joint_space_goal_cb(self, msg: FabricsJointSpaceGoal):
        rospy.loginfo_throttle(5, "Received joint space goal")
        self._goal_msg = msg
        self._goal = self.goal_wrapper.wrap(msg)
        goal_string = self.goal_wrapper.goal_string(self._goal)
        if self._goal_string != goal_string:
            self._changed_planner = True
        self._goal_string = goal_string

    def ee_pose_goal_cb(self, msg: FabricsPoseGoal):
        rospy.loginfo_throttle(5, "Received ee pose goal")
        self._goal_msg = msg
        self._goal = self.goal_wrapper.wrap(msg)
        goal_string = self.goal_wrapper.goal_string(self._goal)
        if self._goal_string != goal_string:
            self._changed_planner = True
        self._goal_string = goal_string

    def constraints_goal_cb(self, msg: FabricsConstraintsGoal):
        rospy.loginfo_throttle(5, "Received constraints goal")
        self._goal_msg = msg
        self._goal = self.goal_wrapper.wrap(msg)
        goal_string = self.goal_wrapper.goal_string(self._goal)
        if self._goal_string != goal_string:
            self._changed_planner = True
        self._goal_string = goal_string

    def cb_goal_rviz(self, msg: PoseStamped):
        rospy.loginfo_throttle(5, "Received rviz goal, weight set in config.yaml file")
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
        x_obst_sphere = np.full((self.num_sphere_obstacles, 3), [100.0] * 3) #todo
        radius_obst = np.full((self.num_sphere_obstacles, 1), 0.05)
        x_obsts_cuboid = np.full((self.num_box_obstacles, 3), [100.0] * 3) #todo
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
            # 'radius_body_base_link': rospy.get_param('/radius_body_base_link') 
        })

    def check_goal_reached(self):
        self.positional_goal_tolerance = rospy.get_param('/positional_goal_tolerance')
        x_goal = self._runtime_arguments['x_goal_0'][0:self._dim_state]
        q = self._runtime_arguments['q']
        x_state = self._forward_kinematics.fk(q, self.root_link, self.end_link, positionOnly=True)[0:self._dim_state]
        distance = self.dist_goal_ee(x_state, x_goal)
        if distance <= self.positional_goal_tolerance and self.goal_reached == False:
            self.q_final = copy.deepcopy(self._q)
            self.goal_reached = True
        elif distance <= self.positional_goal_tolerance:
            self.goal_reached = True
            print("goal pose is reached!!")
        else:
            kkk=1
            #print("distance between end-eff and goal:", distance)

    def dist_goal_ee(self, x_state, x_goal):
        distance = np.linalg.norm(x_state - x_goal)
        return distance

    @abstractmethod
    def set_joint_states_values(self):
        pass

    def compute_action(self):
        self.compose_runtime_obstacles_argument()
        self.goal_wrapper.compose_runtime_arguments(self._goal, self._runtime_arguments)
        self.set_joint_states_values()
        # print("self.goal: ", self._runtime_arguments['x_goal_0'])
        # print("self._runtime_arguments, angle_goal1:", self._runtime_arguments['angle_goal_1'])
        # # print("self._runtime_arguments, weight_goal1:", self._runtime_arguments['weight_goal_1'])
        print("self._runtime_arguments, q:", self._runtime_arguments['q'])
        #print("self._runtime_arguments, qdot:", self._runtime_arguments['qdot'])  
        # print("self._runtime_arguments, x_obst:", self._runtime_arguments['x_obst'])
        if self.goal_reached == False:
            action = self._planner.compute_action(
                **self._runtime_arguments,
            )
        else:
            action = np.zeros_like(self._action)
        #print("action:", action)
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
            self.publish_obstacles()
            action = np.zeros_like(self._action)
            action = self.compute_action()
            self._action = action
            self._action = self._action * alpha + action * (1-alpha)
            self._action = np.clip(self._action, self._min_vel, self._max_vel)
            print("action:", self._action)
            self.publish_action()
            self.check_goal_reached()
            # print("after goal reached!")
        except Exception as e:
            rospy.loginfo(f"Not planning due to error {e}")
            rospy.loginfo("Waiting for correct planner to be loaded or bug is fixed.")
            self.load_planner(self._goal)
