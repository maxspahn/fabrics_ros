from typing import Union
import numpy as np
import rospy

import tf
from fabrics_msgs.msg import (
    FabricsJointSpaceGoal,
    FabricsPoseGoal,
    FabricsConstraintsGoal,
)
from fabrics_msgs.msg import FabricsState
FabricsGoalUnion = Union[FabricsJointSpaceGoal,FabricsPoseGoal,FabricsConstraintsGoal]
from fabrics_bridge.utils import quaternion_to_rotation_matrix
from fabrics_bridge.utils import Quaternion, Point



class FabricsGoalEvaluator(object):
    state: FabricsState
    def __init__(self, fk):
        self.state = FabricsState(
            goal_reached=False
        )
        self._fk = fk
        self._root_link = rospy.get_param('/root_link')
        self._end_effector_link = rospy.get_param('/end_effector_link')
        self._orientation_helper_link = rospy.get_param("/orientation_helper_link")
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.load_parameters()
        self.state_publisher = rospy.Publisher("state", FabricsState, queue_size=10)
        rospy.sleep(1.0)

    def load_parameters(self):
        self.default_positional_goal_tolerance = rospy.get_param("/positional_goal_tolerance")
        self.default_angular_goal_tolerance = rospy.get_param("/angular_goal_tolerance")
    
    def compute_fk(self, q: np.ndarray, parent_link: str, child_link: str) -> np.ndarray:
        if parent_link == self._root_link:
            parent_position = np.zeros(3)
        else:
            parent_position = self._fk.fk(q, parent_link=self._root_link, child_link=parent_link, positionOnly=True)
        child_position = self._fk.fk(q, parent_link=self._root_link, child_link=child_link, positionOnly=True)
        return child_position - parent_position

    def evaluate(self, goal: FabricsGoalUnion, q: np.ndarray) -> bool:
        if rospy.get_param("/robot_type") == "boxer":
            rospy.logwarn("Planning state evaluation not available for boxer robot")
            return False
        """
        if goal.tolerance_goal_0 == 0:
            self.positional_goal_tolerance = self.default_positional_goal_tolerance
        else:
            self.positional_goal_tolerance = goal.tolerance_goal_0
        if goal.tolerance_goal_1 == 0:
            self.angular_goal_tolerance = self.default_angular_goal_tolerance
        else:
            self.angular_goal_tolerance = goal.tolerance_goal_1
        """

        if isinstance(goal, FabricsJointSpaceGoal):
            self.state.tolerances = [goal.tolerance]
            difference_vector = q - np.array(goal.goal_joint_state.position)
            self.state.errors = [float(np.linalg.norm(difference_vector))]
            self.state.goal_reached = (self.state.errors[0] < goal.tolerance)
        elif isinstance(goal, FabricsConstraintsGoal):
            constraints_satisfied = []
            self.state.errors = []
            self.state.tolerances = []
            for constraint in goal.constraints:
                fk = self.compute_fk(q, constraint.parent_link, constraint.child_link)
                error = np.linalg.norm(fk[list(constraint.indices)] - constraint.geometric_constraint.data)
                self.state.errors.append(error)
                self.state.tolerances.append(constraint.tolerance)

                if error < constraint.tolerance:
                    constraints_satisfied.append(True)
                else:
                    constraints_satisfied.append(False)
            self.state.header.stamp = rospy.Time.now()
            self.state.goal_reached = all(constraints_satisfied)

        elif isinstance(goal, FabricsPoseGoal):
            # broadcast goal frame
            # Note: at 10x lower frequency than act loop, because tf
            """
            if not hasattr(self, "eval_i"):
                self.eval_i = 0
            self.eval_i += 1
            if self.eval_i % 10 == 0:
                self.eval_i = 0
                self.tf_broadcaster.sendTransform(
                    list(goal.goal_pose.position),
                    list(goal.goal_pose.orientation),
                    rospy.Time.now(),
                    "/fabrics_goal",
                    rospy.get_param("/root_link"),
                )
            """

            des_position = np.array(list(goal.goal_pose.position))
            goal_quaternion = np.array(list(goal.goal_pose.orientation))
            goal_rotation_matrix = quaternion_to_rotation_matrix(goal_quaternion, ordering='xyzw')

            fk_position = self.compute_fk(q, self._root_link, self._end_effector_link)
            fk_ori = self.compute_fk(q, self._root_link, self._orientation_helper_link)
            fk_orientation = fk_ori - fk_position
            default_goal = np.array([0.0, 0.0, -np.linalg.norm(fk_orientation)])
            des_orientation = np.dot(goal_rotation_matrix, default_goal)

            positional_error = np.linalg.norm(
                fk_position - des_position 
            )
            angular_error = np.linalg.norm(
                fk_orientation - des_orientation
            )
            # update state
            self.state.errors = [positional_error, angular_error]
            self.state.tolerances = [goal.tolerance_position, goal.tolerance_orientation]
            self.state.goal_reached = (
                positional_error < goal.tolerance_position
                and angular_error < goal.tolerance_orientation
            )
        self.state.header.stamp = rospy.Time.now()
        self.state_publisher.publish(self.state)
        return self.state.goal_reached
