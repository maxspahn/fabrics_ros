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

def quaternion_to_rotation_matrix(
    quaternion: np.ndarray, ordering: str = "wxyz"
) -> np.ndarray:
    # Normalize the quaternion if needed
    quaternion /= np.linalg.norm(quaternion)

    if ordering == "wxyz":
        w, x, y, z = quaternion
    elif ordering == "xyzw":
        x, y, z, w = quaternion
    else:
        raise InvalidQuaternionOrderError(
            f"Order {ordering} is not permitted, options are 'xyzw', and 'wxyz'"
        )
    rotation_matrix = np.array(
        [
            [
                1 - 2 * y**2 - 2 * z**2,
                2 * x * y - 2 * w * z,
                2 * x * z + 2 * w * y,
            ],
            [
                2 * x * y + 2 * w * z,
                1 - 2 * x**2 - 2 * z**2,
                2 * y * z - 2 * w * x,
            ],
            [
                2 * x * z - 2 * w * y,
                2 * y * z + 2 * w * x,
                1 - 2 * x**2 - 2 * y**2,
            ],
        ]
    )

    return rotation_matrix



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
                goal_string += str(sub_goal.indices())
        return goal_string

    def hash_goal(self, goal: GoalComposition) -> str:
        goal_type = self.goal_string(goal)
        hash_goal_type = hashlib.md5(goal_type.encode("utf-8")).hexdigest()
        return hash_goal_type


    def compose_pose_goal(self, goal_msg: FabricsPoseGoal):
        goal_position = list(goal_msg.goal_pose.position)
        goal_quaternion = list(goal_msg.goal_pose.orientation)
        goal_rotation_matrix = quaternion_to_rotation_matrix(goal_quaternion, ordering='xyzw')
        default_goal = np.array([0.0, 0.0, 0.1])
        orientation_position = np.dot(goal_rotation_matrix, default_goal)
        goal_dict = {
            "position": {
                "weight": goal_msg.weight_position,
                "is_primary_goal": True,
                "indices": [0, 1, 2], 
                "parent_link": rospy.get_param("/root_link"),
                "child_link": rospy.get_param("/end_effector_link"),
                "desired_position": goal_position,
                "epsilon": 0.01,
                "type": "staticSubGoal",
            },
            "orientation": {
                "weight": goal_msg.weight_orientation,
                "is_primary_goal": False,
                "indices": [0, 1, 2],
                "parent_link": rospy.get_param("/orientation_helper_link"),
                "child_link": rospy.get_param("/end_effector_link"),
                "desired_position": orientation_position.tolist(),
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
        goal_dict = {}
        for i, constraint in enumerate(goal_msg.constraints):
            desired_position = constraint.geometric_constraint.data
            if isinstance(desired_position, np.ndarray):
                desired_position = desired_position.tolist()

            sub_goal_dict = {
                    "weight": constraint.weight,
                    "is_primary_goal": False,
                    "indices": constraint.indices,
                    "parent_link": constraint.parent_link,
                    "child_link": constraint.child_link,
                    "desired_position": desired_position,
                    "epsilon": constraint.tolerance,
                    "type": "staticSubGoal",
            }
            if i == 0:
                sub_goal_dict['is_primary_goal'] = True
            goal_dict[f"sub_goal_{i}"] = sub_goal_dict
        return GoalComposition(name="goal", content_dict=goal_dict)

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
