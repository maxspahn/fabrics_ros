from typing import Union, List
import hashlib
from mpscenes.goals.sub_goal_creator import StaticSubGoal
import rospy
from fabrics_msgs.msg import (
        FabricsJointSpaceGoal,
        FabricsConstraintsGoal,
        FabricsPoseGoal,
)
import numpy as np
import tf
from mpscenes.goals.goal_composition import GoalComposition
from mpscenes.goals.sub_goal import SubGoal
from geometry_msgs.msg import Point, PoseStamped, Quaternion


FabricsGoalUnion = Union[FabricsJointSpaceGoal,FabricsPoseGoal,FabricsConstraintsGoal]

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

    def wrap(self, goal_msg: FabricsGoalUnion) -> bool:
        changed_planner: bool = False
        goal_type = type(goal_msg)
        if not self._goal_type == goal_type:
            self._goal_type = goal_type.__name__
            changed_planner = True
        if isinstance(goal_msg, FabricsPoseGoal):
            return self.compose_pose_goal(goal_msg), changed_planner
        elif isinstance(goal_msg, FabricsJointSpaceGoal):
            return self.compose_joint_space_goal(goal_msg), changed_planner
        elif isinstance(goal_msg, FabricsConstraintsGoal):
            return self.compose_constraints_goal(goal_msg), changed_planner
        else:
            raise InvalidGoalError()

    def goal_string(self, goal: GoalComposition) -> str:
        sub_goals: List[SubGoal] = goal.sub_goals()
        goal_string = ""
        for sub_goal in sub_goals:
            goal_string += sub_goal.name()
            if isinstance(sub_goal, StaticSubGoal):
                goal_string += sub_goal.parent_link()
                goal_string += sub_goal.child_link()
        return goal_string

    def hash_goal(self, goal: GoalComposition) -> str:
        goal_type = self.goal_string(goal)
        hash_goal_type = hashlib.md5(goal_type.encode("utf-8")).hexdigest()
        return hash_goal_type


    def compose_pose_goal(self, goal_msg: FabricsPoseGoal):
        indices = rospy.get_param('/goal_indices')
        goal_position = [list(goal_msg.goal_pose.position)[i] for i in indices]
        goal_dict = {
            "position": {
                "weight": goal_msg.weight_position,
                "is_primary_goal": True,
                "indices": indices, 
                "parent_link": rospy.get_param("/root_link"),
                "child_link": rospy.get_param("/end_effector_link"),
                "desired_position": goal_position,
                "epsilon": 0.01,
                "type": "staticSubGoal",
            },
            "orientation": {
                "weight": goal_msg.weight_orientation,
                "is_primary_goal": False,
                "indices": [0, 1],
                "parent_link": rospy.get_param("/orientation_helper_link"),
                "child_link": rospy.get_param("/end_effector_link"),
                "angle": list(goal_msg.goal_pose.orientation),
                "desired_position": [0.0, 0.0],
                "epsilon": 0.01,
                "type": "staticSubGoal",
            },
        }
        return GoalComposition(name="goal", content_dict=goal_dict)

    def compose_joint_space_goal(self, goal_msg: FabricsJointSpaceGoal):
        joint_positions = list(goal_msg.goal_joint_state.position)
        dimension = len(joint_positions)
        goal_dict = {
            "joint_position": {
                "weight": goal_msg.weight,
                "is_primary_goal": True,
                "indices": list(range(dimension)),
                "desired_position": joint_positions,
                "epsilon": 0.01,
                "type": "staticJointSpaceSubGoal",
            }
        }
        return GoalComposition(name="goal", content_dict=goal_dict)

    def compose_constraints_goal(self, goal_msg: FabricsConstraintsGoal):
        pass

    def compose_dummy_goal(self, goal_type: str):
        if goal_type == 'FabricsJointSpaceGoal':
            goal_msg = FabricsJointSpaceGoal()
            goal_msg.goal_joint_state.position = [0, ] * rospy.get_param("/degrees_of_freedom")
            return self.compose_joint_space_goal(goal_msg)
        else:
            raise InvalidGoalError(f"Cannot create dummy goal for goal type {goal_type}")

        

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
