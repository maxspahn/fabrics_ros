from typing import Union
import tf
import rospy
from fabrics_msgs.msg import FabricsState

#!/usr/bin/env python3
import numpy as np
import rospy

import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, PoseStamped
from fabrics_msgs.msg import (
    FabricsJointSpaceGoal,
    FabricsPoseGoal,
    FabricsConstraintsGoal,
)
FabricsGoalUnion = Union[FabricsJointSpaceGoal,FabricsPoseGoal,FabricsConstraintsGoal]


class FabricsGoalEvaluator(object):
    state: FabricsState
    def __init__(self, fk):
        self.state = FabricsState(
            goal_reached=False
        )
        self._fk = fk
        self._root_link = rospy.get_param('/root_link')
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.load_parameters()
        self.state_publisher = rospy.Publisher("state", FabricsState, queue_size=10)
        rospy.sleep(1.0)

    def load_parameters(self):
        self.default_positional_goal_tolerance = rospy.get_param("/positional_goal_tolerance")
        self.default_angular_goal_tolerance = rospy.get_param("/angular_goal_tolerance")
    
    def compute_fk(self, q: np.ndarray, parent_link: str, child_link: str) -> np.ndarray:
        parent_position = self._fk.fk(q, self._root_link, parent_link, positionOnly=True)
        child_position = self._fk.fk(q, self._root_link, child_link, positionOnly=True)
        return child_position - parent_position

    def evaluate(self, goal: FabricsGoalUnion, joint_states: JointState) -> bool:
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
            difference_vector = np.array(joint_states.position) - np.array(goal.goal_joint_state.position)
            self.state.errors = [float(np.linalg.norm(difference_vector))]
            self.state.goal_reached = (self.state.errors[0] < goal.tolerance)
        elif isinstance(goal, FabricsConstraintsGoal):
            constraints_satisfied = []
            q = joint_states.position
            self.state.errors = []
            self.state.tolerances = []
            for constraint in goal.constraints:
                fk = self.compute_fk(q, constraint.parent_link, constraint.child_link)
                error = np.linalg.norm(fk - constraint.geometric_constraint.data)
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

            # evaluate positional_error
            trans, _ = self.tf_listener.lookupTransform(
                rospy.get_param("/root_link"), rospy.get_param("/end_effector_link"), rospy.Time(0)
            )
            positional_error = np.linalg.norm(
                np.array(trans) - list(goal.goal_pose.position)
            )

            # evaluate angular_error
            # Note: this is done by taking the L2 norm between a point at z=0.5 along the '/panda_link8' frame and transforming it to the goal frame
            p1 = PoseStamped()
            p1.header.frame_id = rospy.get_param("/end_effector_link")
            p1.pose.position = Point(x=0, y=0, z=0.5)
            try:
                p2 = self.tf_listener.transformPose("/fabrics_goal", p1)
                angular_error = np.linalg.norm(
                    np.array(list(p1.pose.position)) - list(p2.pose.position)
                )
            except Exception as e:
                angular_error = 10
                rospy.loginfo(e)
                rospy.loginfo("Waiting for tf to be on time.")

            # update state
            self.state.errors = [positional_error, angular_error]
            self.state.tolerances = [goal.tolerance_position, goal.tolerance_orientation]
            self.state.goal_reached = (
                positional_error < goal.tolerance_position
                and angular_error < goal.tolerance_orientation
            )
        self.state.header.stamp = rospy.Time.now()
        #self.state.tolerance_goal_0 = self.positional_goal_tolerance
        #self.state.tolerance_goal_1 = self.angular_goal_tolerance
        self.state_publisher.publish(self.state)
        return self.state.goal_reached
