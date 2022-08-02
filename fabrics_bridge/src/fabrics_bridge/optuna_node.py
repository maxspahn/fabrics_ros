# ros imports
from abc import abstractmethod
import rospy
import rospkg
import optuna
import joblib
from fabrics_msgs.msg import FabricsGoal, FabricsObstacleArray, FabricsObstacle, FabricsState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import Empty

from pynput.keyboard import Listener,KeyCode

from fabrics_bridge.rectangle_spheres import *


class OptunaNode(object):
    def __init__(self):
        rospy.init_node("client_node")
        ros_pack = rospkg.RosPack()
        package_path = ros_pack.get_path('fabrics_bridge')
        try:
            self._study_file = package_path + '/studies/' + rospy.get_param('/optuna/study_file')
        except Exception as e:
            self._study_file = None
        self._name_map = {}
        self._name_map['exp_fin_limit_leaf'] = 'limit/fin/exp'
        self._name_map['k_fin_limit_leaf'] = 'limit/fin/k'
        self._name_map['exp_fin_obst_leaf'] = 'collision_avoidance/fin/exp'
        self._name_map['k_fin_obst_leaf'] = 'collision_avoidance/fin/k'
        self._name_map['exp_fin_self_leaf'] = 'self_collision_avoidance/fin/exp'
        self._name_map['k_fin_self_leaf'] = 'self_collision_avoidance/fin/k'
        self._name_map['exp_geo_limit_leaf'] = 'limit/geo/exp'
        self._name_map['k_geo_limit_leaf'] = 'limit/geo/k'
        self._name_map['exp_geo_obst_leaf'] = 'collision_avoidance/geo/exp'
        self._name_map['k_geo_obst_leaf'] = 'collision_avoidance/geo/k'
        self._name_map['exp_geo_self_leaf'] = 'self_collision_avoidance/geo/exp'
        self._name_map['k_geo_self_leaf'] = 'self_collision_avoidance/geo/k'
        self._name_map['alpha_b_damper'] = 'damper/alpha_b'
        self._name_map['beta_close_damper'] = 'damper/beta_close'
        self._name_map['beta_distant_damper'] = 'damper/beta_distant'
        self._name_map['radius_shift_damper'] = 'damper/radius_shift'
        self._number_trials = rospy.get_param('/optuna/number_trials')
        self._maximum_steps = rospy.get_param('/optuna/maximum_time_steps')
        self._weights = rospy.get_param('/optuna/weights')
        rospy.loginfo(self._weights)

        self._rate = rospy.Rate(10)
        # if param num_obs is not set, default number of obstacles is 2
        self.init_connections()
        self.initialize_study()
        self.obs = FabricsObstacleArray()
        self.obs.obstacles = []
        origin = [-0.4, 0.0, 0.33]
        panda_limits = np.array([
                [-2.8973, 2.8973],
                [-1.7628, 1.7628],
                [-2.8974, 2.8973],
                [-3.0718, -0.0698],
                [-2.8973, 2.8973],
                [2.0175, 3.7525],
                [-2.8973, 2.8973]
            ])
        self._lower_limits = 1.0 * np.array(panda_limits[:, 0])
        self._upper_limits = 1.0 * np.array(panda_limits[:, 1])

        self._key_listener = Listener(on_press=self._on_press)
        self._key_listener.start()
        self.initialize_home_goal()
        self._joint_positions = np.array(7)
        self._old_joint_positions = np.array(7)
        self._stopped_study = False


    def initialize_home_goal(self):
        self._home_goal = FabricsGoal()
        self._home_goal.goal_joint_state.header.stamp = rospy.Time.now()
        self._home_goal.goal_joint_state.name = [f"panda_joint{i}" for i in range(7)]
        #self._home_goal.goal_joint_state.position = [0.0, -0.9, 0.0, -1.501, 0.0, 1.8675, 0.0]
        self._home_goal.goal_joint_state.position = [0.0, -0.9, 0.0, -1.501, 0.0, 1.8675, np.pi/ 4]
        self._home_goal.goal_type = "joint_space"
        self._home_goal.weight_goal_0 = 3.0 * 0.5
        self._home_goal.tolerance_goal_0 = 0.02

    def _on_press(self, key):
        if key == KeyCode.from_char('e'):
            rospy.loginfo("Ending trial due to user call.")
            self._manually_ended = True
        if key == KeyCode.from_char('s'):
            rospy.loginfo("Stopping study due to user call.")
            self._stopped_study = True

    def initialize_study(self):
        rospy.loginfo(f"Initialize study from file {self._study_file}")
        if self._study_file:
            self._study = joblib.load(self._study_file)
            rospy.loginfo(f"Loaded study from {self._study_file}")
        else:
            self._study = optuna.create_study(direction="minimize")
            rospy.loginfo("Created new study")
        
    def init_connections(self):
        self._goal_publisher = rospy.Publisher(
            "planning_goal", FabricsGoal, queue_size=10
        )
        self._obs_publisher = rospy.Publisher(
            "planning_obs", FabricsObstacleArray, queue_size=10
        )
        self._state_sub = rospy.Subscriber(
            "state", FabricsState, self.state_cb
        )
        self._joint_state_sub = rospy.Subscriber(
            "joint_states_filtered", JointState, self.joint_state_cb
        )
        self._parameter_pub = rospy.Publisher(
            "set_parameters", Empty, queue_size=10,
        )

    def joint_state_cb(self, state: JointState):
        self._joint_positions = np.array(state.position[5:])

    def state_cb(self, state: FabricsState):
        if state.goal_reached:
            self._goal_reached += state.goal_reached
        else:
            self._goal_reached = 0

    def reset_parameters(self):
        rospy.set_param('/fabrics_geometries/limit/fin/exp', 1.0)
        rospy.set_param('/fabrics_geometries/limit/fin/k', 0.1)
        rospy.set_param('/fabrics_geometries/limit/geo/exp', 1.0)
        rospy.set_param('/fabrics_geometries/limit/geo/k', 0.1)
        rospy.set_param('/fabrics_geometries/collision_avoidance/fin/exp', 1.0)
        rospy.set_param('/fabrics_geometries/collision_avoidance/fin/k', 0.1)
        rospy.set_param('/fabrics_geometries/collision_avoidance/geo/exp', 1.0)
        rospy.set_param('/fabrics_geometries/collision_avoidance/geo/k', 0.1)
        rospy.set_param('/fabrics_geometries/self_collision_avoidance/fin/exp', 1.0)
        rospy.set_param('/fabrics_geometries/self_collision_avoidance/fin/k', 1.0)
        rospy.set_param('/fabrics_geometries/self_collision_avoidance/geo/exp', 1.0)
        rospy.set_param('/fabrics_geometries/self_collision_avoidance/geo/k', 1.0)
        rospy.set_param('/fabrics_geometries/base_inertia', 0.5)

    def return_home(self):
        self._goal_reached = 0
        rospy.loginfo("Returning home")
        self.reset_parameters()
        self._parameter_pub.publish(Empty())
        while self._goal_reached < 100:
            self._goal_publisher.publish(self._home_goal)
            self._rate.sleep()
        joblib.dump(self._study, "temp_optuna_file.pkl")

    def sample_parameter(self, trial, name, limits, param_is_int = False, log=False):
        parameter_name = '/fabrics_geometries'
        fabrics_name = ''
        for i in name:
            parameter_name += f'/{i}'
            fabrics_name += f'_{i}'
        if param_is_int:
            rospy.set_param(parameter_name, trial.suggest_int(fabrics_name, limits[0], limits[1], log=log))
        else:
            rospy.set_param(parameter_name, trial.suggest_float(fabrics_name, limits[0], limits[1], log=log))

    def set_parameter(self, name, value):
        if name in list(self._name_map.keys()):
            mapped_name = self._name_map[name]
        else:
            mapped_name = name
        rospy.loginfo(f"Setting param {mapped_name} to {value}")
        parameter_name = '/fabrics_geometries' + '/' + mapped_name
        rospy.set_param(parameter_name, value)

    def select_best_params(self):
        for param_name, param_value in self._study.best_params.items():
            self.set_parameter(param_name, param_value)
        self._parameter_pub.publish(Empty())


    def sample_fabrics_params(self, trial):
        self.sample_parameter(trial, ['collision_avoidance', 'fin', 'exp'], [1, 5], param_is_int=True)
        self.sample_parameter(trial, ['collision_avoidance', 'fin', 'k'], [0.1, 0.5], param_is_int=False, log=True)
        self.sample_parameter(trial, ['collision_avoidance', 'geo', 'exp'], [1, 5], param_is_int=True)
        self.sample_parameter(trial, ['collision_avoidance', 'geo', 'k'], [0.1, 0.5], param_is_int=False, log=True)
        self.sample_parameter(trial, ['limit', 'fin', 'exp'], [1, 5], param_is_int=True)
        self.sample_parameter(trial, ['limit', 'fin', 'k'], [0.01, 0.5], param_is_int=False, log=True)
        self.sample_parameter(trial, ['limit', 'geo', 'exp'], [1, 5], param_is_int=True)
        self.sample_parameter(trial, ['limit', 'geo', 'k'], [0.01, 0.5], param_is_int=False, log=True)
        self.sample_parameter(trial, ['self_collision_avoidance', 'fin', 'exp'], [1, 5], param_is_int=True)
        self.sample_parameter(trial, ['self_collision_avoidance', 'fin', 'k'], [0.1, 0.5], param_is_int=False, log=True)
        self.sample_parameter(trial, ['self_collision_avoidance', 'geo', 'exp'], [1, 5], param_is_int=True)
        self.sample_parameter(trial, ['self_collision_avoidance', 'geo', 'k'], [0.1, 0.5], param_is_int=False, log=True)
        self.sample_parameter(trial, ['damper', 'alpha_b'], [0.0, 1.0]) 
        self.sample_parameter(trial, ['damper', 'radius_shift'], [0.01, 0.1]) 
        self.sample_parameter(trial, ['damper', 'beta_close'], [5., 20.0]) 
        self.sample_parameter(trial, ['damper', 'beta_distant'], [0.01, 0.1]) 
        self.sample_parameter(trial, ['base_inertia'], [0, 1.00])
        self._parameter_pub.publish(Empty())

    @abstractmethod
    def publish_goal():
        pass


    def objective(self, trial):
        self.return_home()
        self.sample_fabrics_params(trial)
        self._manually_ended = False
        self.publish_goal()
        # Initialize costs
        initial_distance_to_goal = np.linalg.norm(self._home_goal.goal_joint_state.position - self.goal.goal_joint_state.position)
        path_length = 0.0
        distance_to_goal = 0.0
        for number_steps in range(self._maximum_steps):
            if number_steps % 10000 == 0:
                rospy.loginfo(f"Ran {number_steps} steps")
            self._rate.sleep()
            distance_to_goal += np.linalg.norm(self._joint_positions - self.goal.goal_joint_state.position)/initial_distance_to_goal
            path_length += np.linalg.norm(self._joint_positions - self._old_joint_positions)
            self._old_joint_positions = self._joint_positions
            if self._manually_ended:
                return 100
        costs = {
            "path_length": path_length/initial_distance_to_goal,
            "time_to_goal": distance_to_goal/number_steps,
        }
        return self.total_costs(costs)

    def total_costs(self, costs):
        return sum([self._weights[i] * costs[i] for i in self._weights])

    def stopped(self):
        return self._stopped_study


    def tune(self):
        self._study.optimize(lambda trial: self.objective(trial), n_trials=self._number_trials)

    def test(self):
        costs_per_run = []
        for i in range(self._number_trials):
            cost_run_i = self.objective()
            rospy.loginfo(f"Cost for run {i} : {cost_run_i}")
            costs_per_run.append(cost_run_i)
        rospy.loginfo(costs_per_run)


    def run(self):
        result = self.test()
        rospy.loginfo(f"Test result was {result}")
