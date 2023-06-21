import rospy
from fabrics_msgs.msg import FabricsGoal
import numpy as np
import tf
from mpscenes.goals.goal_composition import GoalComposition
from geometry_msgs.msg import Point, PoseStamped, Quaternion

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

class InvalidGoalError(Exception):
    pass

class FabricsGoalWrapper(object):
    """Wrapper for translation of ros message type FabricsGoal to a fabrics goal"""
    def __init__(self):
        self._goal_type = ""

    def wrap(self, goal_msg: FabricsGoal) -> bool:
        changed_planner: bool = False
        if not self._goal_type == goal_msg.goal_type:
            self._goal_type = goal_msg.goal_type
            changed_planner = True
        return self.compose_goal(goal_msg), changed_planner

    def compose_goal(self, goal_msg: FabricsGoal):
        if goal_msg.goal_type == "ee_pose":
            indices = rospy.get_param('/goal_indices')
            goal_position = [list(goal_msg.goal_pose.pose.position)[i] for i in indices]
            goal_dict = {
                "position": {
                    "weight": goal_msg.weight_goal_0,
                    "is_primary_goal": True,
                    "indices": indices, 
                    "parent_link": rospy.get_param("/root_link"),
                    "child_link": rospy.get_param("/end_effector_link"),
                    "desired_position": goal_position,
                    "epsilon": 0.01,
                    "type": "staticSubGoal",
                },
                "orientation": {
                    "weight": goal_msg.weight_goal_1,
                    "is_primary_goal": False,
                    "indices": [0, 1],
                    "parent_link": rospy.get_param("/orientation_helper_link"),
                    "child_link": rospy.get_param("/end_effector_link"),
                    "angle": list(goal_msg.goal_pose.pose.orientation),
                    "desired_position": [0.0, 0.0],
                    "epsilon": 0.01,
                    "type": "staticSubGoal",
                },
            }
        elif goal_msg.goal_type == "joint_space":
            joint_positions = goal_msg.goal_joint_state.position
            dimension = len(joint_positions)
            goal_dict = {
                "joint_position": {
                    "weight": goal_msg.weight_goal_0,
                    "is_primary_goal": True,
                    "indices": list(range(dimension)),
                    "desired_position": joint_positions,
                    "epsilon": 0.01,
                    "type": "staticJointSpaceSubGoal",
                }
            }
        else:
            raise InvalidGoalError(f"Goal of type {goal_msg.goal_type} is not known")

        return GoalComposition(name="goal", content_dict=goal_dict)

    def compose_dummy_goal(self, goal_type: str):
        goal_msg = FabricsGoal()
        goal_msg.goal_type = goal_type
        if goal_type == 'joint_space':
            goal_msg.goal_joint_state.position = [0, ] * rospy.get_param("/degrees_of_freedom")
        return self.compose_goal(goal_msg)
        

    def compose_runtime_arguments(self, goal: GoalComposition, runtime_arguments: dict) -> None:
        for i, sub_goal in enumerate(goal.sub_goals()):
            runtime_arguments[f'x_goal_{i}'] = np.array(sub_goal.position())
            runtime_arguments[f'weight_goal_{i}'] = 0.5 * np.array([sub_goal.weight()])
            if hasattr(sub_goal, 'angle') and sub_goal.angle():
                euler_goal = list(
                    tf.transformations.euler_from_quaternion(
                        sub_goal.angle(), 'rxyz'
                    )
                )
                rot_mat = tf.transformations.euler_matrix(
                    -euler_goal[1], -euler_goal[0], 0, "ryxz"
                )[:3, :3]
                runtime_arguments[f'angle_goal_{i}'] = rot_mat
