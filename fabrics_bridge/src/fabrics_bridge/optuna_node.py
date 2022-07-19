# ros imports
import rospy
import optuna
import joblib
from fabrics_msgs.msg import FabricsGoal, FabricsObstacleArray, FabricsObstacle, FabricsState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from pynput.keyboard import Listener,KeyCode

from fabrics_bridge.rectangle_spheres import *


class OptunaNode(object):
    def __init__(self, study_file: str = None):
        self._study_file = study_file
        rospy.init_node("client_node")
        self._weights = {"path_length": 0.3, "time_to_goal": 0.7}
        self._rate = rospy.Rate(100)
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
        width = 0.34
        height = 0.52
        inflation_r = 0.05
        extra_spheres = 0
        rec_obs = rectangle_spheres(origin, width, height, inflation_r, extra_spheres)
        self._maximum_steps = 1000000
        self._number_trials = 10
        for obs in rec_obs:
            o = FabricsObstacle()
            o.header.frame_id = "panda_link0"
            o.radius = inflation_r
            o.position = Point(
                x=obs[0],
                y=obs[1],
                z=obs[2],
            )

            self.obs.obstacles.append(o)

        self._key_listener = Listener(on_press=self._on_press)
        self._key_listener.start()
        self.initialize_home_goal()
        self._joint_positions = np.array(7)
        self._old_joint_positions = np.array(7)

    def sample_fabrics_params(self, trial):
        weight_attractor = trial.suggest_float('w', 0.1, 2, log=False)
        return {'attractor_weight': weight}

    def initialize_home_goal(self):
        self._home_goal = FabricsGoal()
        self._home_goal.goal_joint_state.header.stamp = rospy.Time.now()
        self._home_goal.goal_joint_state.name = [f"panda_joint{i}" for i in range(7)]
        #self._home_goal.goal_joint_state.position = [0.0, -0.9, 0.0, -1.501, 0.0, 1.8675, 0.0]
        self._home_goal.goal_joint_state.position = [0.0, -0.9, 0.0, -1.501, 0.0, 1.8675, np.pi/ 4]
        self._home_goal.goal_type = "joint_space"
        self._home_goal.weight_goal_0 = 0.5
        self._home_goal.tolerance_goal_0 = 0.02

    def _on_press(self, key):
        if key == KeyCode.from_char('e'):
            rospy.loginfo("Ending trial due to user call.")
            self._manually_ended = True

    def initialize_study(self):
        if self._study_file:
            self._study = joblib.load(self._study_file)
        else:
            self._study = optuna.create_study(direction="minimize")
        
    def init_connections(self):
        self._goal_publisher = rospy.Publisher(
            "fabrics/planning_goal", FabricsGoal, queue_size=10
        )
        self._obs_publisher = rospy.Publisher(
            "fabrics/planning_obs", FabricsObstacleArray, queue_size=10
        )
        self._state_sub = rospy.Subscriber(
            "fabrics/state", FabricsState, self.state_cb
        )
        self._joint_state_sub = rospy.Subscriber(
            "joint_states_filtered", JointState, self.joint_state_cb
        )

    def joint_state_cb(self, state: JointState):
        self._joint_positions = np.array(state.position[5:])

    def state_cb(self, state: FabricsState):
        if state.goal_reached:
            self._goal_reached += state.goal_reached
        else:
            self._goal_reached = 0

    def reset_parameters(self):
        rospy.set_param('/fabrics_geometries/limit/fin/exp', 0.1)
        rospy.set_param('/fabrics_geometries/limit/fin/k', 0.1)
        rospy.set_param('/fabrics_geometries/limit/geo/exp', 0.1)
        rospy.set_param('/fabrics_geometries/limit/geo/k', 0.1)
        rospy.set_param('/fabrics_geometries/collision_avoidance/fin/exp', 0.1)
        rospy.set_param('/fabrics_geometries/collision_avoidance/fin/k', 0.1)
        rospy.set_param('/fabrics_geometries/collision_avoidance/geo/exp', 0.1)
        rospy.set_param('/fabrics_geometries/collision_avoidance/geo/k', 0.1)
        rospy.set_param('/fabrics_geometries/self_collision_avoidance/fin/exp', 0.1)
        rospy.set_param('/fabrics_geometries/self_collision_avoidance/fin/k', 0.1)
        rospy.set_param('/fabrics_geometries/self_collision_avoidance/geo/exp', 0.1)
        rospy.set_param('/fabrics_geometries/self_collision_avoidance/geo/k', 0.1)
        rospy.set_param('/fabrics_geometry/base_inertia', 0.5)

    def publish_goal(self, weight):
        self.reset_parameters()
        self.goal = FabricsGoal()
        self.goal.goal_joint_state.header.stamp = rospy.Time.now()
        self.goal.goal_joint_state.name = [f"panda_joint{i}" for i in range(7)]
        self.goal.goal_joint_state.position = np.random.random(7) * (self._upper_limits - self._lower_limits) + self._lower_limits
        self.goal.goal_type = "joint_space"
        self.goal.weight_goal_0 = weight
        self.goal.tolerance_goal_0 = 0.02
        self._goal_reached = 0
        for _ in range(10):
            self._goal_publisher.publish(self.goal)

    def return_home(self):
        rospy.set_param('/fabrics/limit_finsler_lambda', 0.1)
        self._goal_reached = 0
        rospy.loginfo("Returning home")
        while self._goal_reached < 100:
            self._goal_publisher.publish(self._home_goal)
            self._rate.sleep()
        joblib.dump(self._study, "temp_optuna_file.pkl")

    def sample_parameter(self, trial, name, limits, param_is_int = False):
        if param_is_int:
            rospy.set_param(f'/fabrics_geometries/{name[0]}/{name[1]}/{name[2]}', trial.suggest_int(f'{name[0]}_{name[1]}_{name[2]}', limits[0], limits[1], log=False))
        else:
            rospy.set_param(f'/fabrics_geometries/{name[0]}/{name[1]}/{name[2]}', trial.suggest_float(f'{name[0]}_{name[1]}_{name[2]}', limits[0], limits[1], log=False))

    def sample_fabrics_params(self, trial):
        self.sample_parameter(trial, ['limit', 'fin', 'exp'], [1, 5], param_is_int=True)
        self.sample_parameter(trial, ['limit', 'fin', 'k'], [1, 5], param_is_int=False)
        self.sample_parameter(trial, ['limit', 'geo', 'exp'], [1, 5], param_is_int=True)
        self.sample_parameter(trial, ['limit', 'geo', 'k'], [1, 5], param_is_int=False)
        self.sample_parameter(trial, ['collision_avoidance', 'fin', 'exp'], [1, 5], param_is_int=True)
        self.sample_parameter(trial, ['collision_avoidance', 'fin', 'k'], [1, 5], param_is_int=False)
        self.sample_parameter(trial, ['collision_avoidance', 'geo', 'exp'], [1, 5], param_is_int=True)
        self.sample_parameter(trial, ['collision_avoidance', 'geo', 'k'], [1, 5], param_is_int=False)
        self.sample_parameter(trial, ['self_collision_avoidance', 'fin', 'exp'], [1, 5], param_is_int=True)
        self.sample_parameter(trial, ['self_collision_avoidance', 'fin', 'k'], [1, 5], param_is_int=False)
        self.sample_parameter(trial, ['self_collision_avoidance', 'geo', 'exp'], [1, 5], param_is_int=True)
        self.sample_parameter(trial, ['self_collision_avoidance', 'geo', 'k'], [1, 5], param_is_int=False)
        rospy.set_param('/fabrics_geometries/base_inertia', trial.suggest_float('base_inertia', 0, 1, log=False))
        weight = trial.suggest_float('w', 0.1, 2, log=False)
        return {'attractor_weight': weight}
            


    def objective(self, trial):
        self.return_home()
        params = self.sample_fabrics_params(trial)
        weight = params['attractor_weight']
        self._manually_ended = False
        self.publish_goal(weight)
        self._goal_reached = 0
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
            if self._goal_reached > 100:
                break
            if self._manually_ended:
                return 100
        costs = {
            "path_length": path_length/initial_distance_to_goal,
            "time_to_goal": distance_to_goal/number_steps,
        }
        return self.total_costs(costs)

    def total_costs(self, costs):
        return sum([self._weights[i] * costs[i] for i in self._weights])


    def run(self):
        self._study.optimize(lambda trial: self.objective(trial), n_trials=self._number_trials)
