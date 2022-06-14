import rospy
from fabrics_msgs.msg import FabricsGoal
import numpy as np
import tf
from MotionPlanningGoal.goalComposition import GoalComposition
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
            goal_dict = {
                "position": {
                    "m": 3,
                    "w": goal_msg.weight_goal_0,
                    "prime": True,
                    "indices": [0, 1, 2],
                    "parent_link": "panda_link0",
                    "child_link": "panda_vacuum",
                    "desired_position": list(goal_msg.goal_pose.pose.position),
                    "epsilon": 0.01,
                    "type": "staticSubGoal",
                },
                "orientation": {
                    "m": 2,
                    "w": goal_msg.weight_goal_1,
                    "prime": False,
                    "indices": [0, 1],
                    "parent_link": "panda_hand",
                    "child_link": "panda_vacuum",
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
                    "m": dimension,
                    "w": goal_msg.weight_goal_0,
                    "prime": True,
                    "indices": list(range(dimension)),
                    "desired_position": joint_positions,
                    "epsilon": 0.01,
                    "type": "staticJointSpaceSubGoal",
                }
            }
        else:
            raise InvalidGoalError(f"Goal of type {goal_msg.goal_type} is not known")

        return GoalComposition(name="goal", contentDict=goal_dict)

    def compose_dummy_goal(self, goal_type: str):
        goal_msg = FabricsGoal()
        goal_msg.goal_type = goal_type
        if goal_type == 'joint_space':
            goal_msg.goal_joint_state.position = [0, ] * 7
        return self.compose_goal(goal_msg)
        

    def compose_runtime_arguments(self, goal: GoalComposition) -> dict:
        goal_args = {}
        for i, sub_goal in enumerate(goal.subGoals()):
            goal_args[f'x_goal_{i}'] = np.array(sub_goal.position())
            goal_args[f'weight_goal_{i}'] = np.array([sub_goal.weight()])
            if hasattr(sub_goal, 'angle') and sub_goal.angle():
                euler_goal = list(
                    tf.transformations.euler_from_quaternion(
                        sub_goal.angle(), 'rxyz'
                    )
                )
                rot_mat = tf.transformations.euler_matrix(
                    -euler_goal[1], -euler_goal[0], 0, "ryxz"
                )[:3, :3]
                goal_args[f'angle_goal_{i}'] = rot_mat
        return goal_args
