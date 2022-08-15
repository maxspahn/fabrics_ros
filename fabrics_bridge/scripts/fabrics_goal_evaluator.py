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
    FabricsGoal,
)

from forwardkinematics.urdfFks.boxerFk import BoxerFk


class FabricsGoalEvaluator(object):
    def __init__(self):
        self.state = FabricsState(
            goal_reached=False, positional_error=1, angular_error=1
        )
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.load_parameters()
        self.state_publisher = rospy.Publisher("state", FabricsState, queue_size=10)
        rospy.sleep(1.0)
        self._boxer_fk = BoxerFk()

    def load_parameters(self):
        self.default_positional_goal_tolerance = rospy.get_param("/positional_goal_tolerance")
        self.default_angular_goal_tolerance = rospy.get_param("/angular_goal_tolerance")

    def evaluate(self, goal: FabricsGoal, joint_states: JointState) -> bool:
        if goal.tolerance_goal_0 == 0:
            self.positional_goal_tolerance = self.default_positional_goal_tolerance
        else:
            self.positional_goal_tolerance = goal.tolerance_goal_0
        if goal.tolerance_goal_1 == 0:
            self.angular_goal_tolerance = self.default_angular_goal_tolerance
        else:
            self.angular_goal_tolerance = goal.tolerance_goal_1
        if rospy.get_param("/robot_type") == "boxer":
            fk = self._boxer_fk.fk_by_name(joint_states.position, "ee_link", positionOnly=True)
            self.state.positional_error = np.linalg.norm(
                np.array(fk[0:2]) - np.array([goal.goal_pose.pose.position.x, goal.goal_pose.pose.position.y])
            )
            self.state.angular_error = 0.0
            self.state.goal_reached = (
                self.state.positional_error < self.positional_goal_tolerance
            )
            self.state.tolerance_goal_0 = self.positional_goal_tolerance
            self.state.tolerance_goal_1 = self.angular_goal_tolerance
            self.state_publisher.publish(self.state)
            return self.state.goal_reached
        if len(joint_states.position) == 0:
            rospy.logwarn("Not receiving joint states in evaluator.")
            return False
        if goal.goal_type == 'joint_space':
            if rospy.get_param("/robot_type") == "albert":
                error_vector_base = np.array(joint_states.position[:2]) - np.array(goal.goal_joint_state.position[:2])
                error_vector_arm = np.array(joint_states.position[3:]) - np.array(goal.goal_joint_state.position[3:])
                self.state.positional_error = np.linalg.norm(error_vector_arm) + np.linalg.norm(error_vector_base)
            else:
                self.state.positional_error = np.linalg.norm(
                    np.array(joint_states.position) - np.array(goal.goal_joint_state.position)
                )
            self.state.angular_error = 0.0
            self.state.goal_reached = (
                self.state.positional_error < self.positional_goal_tolerance
            )
        elif goal.goal_type == 'ee_pose':
            # broadcast goal frame
            # Note: at 10x lower frequency than act loop, because tf
            if not hasattr(self, "eval_i"):
                self.eval_i = 0
            self.eval_i += 1
            if self.eval_i % 10 == 0:
                self.eval_i = 0
                self.tf_broadcaster.sendTransform(
                    list(goal.goal_pose.pose.position),
                    list(goal.goal_pose.pose.orientation),
                    rospy.Time.now(),
                    "/fabrics_goal",
                    rospy.get_param("/root_link"),
                )

            # evaluate positional_error
            trans, _ = self.tf_listener.lookupTransform(
                rospy.get_param("/root_link"), rospy.get_param("/end_effector_link"), rospy.Time(0)
            )
            self.state.positional_error = np.linalg.norm(
                np.array(trans) - list(goal.goal_pose.pose.position)
            )

            # evaluate angular_error
            # Note: this is done by taking the L2 norm between a point at z=0.5 along the '/panda_link8' frame and transforming it to the goal frame
            p1 = PoseStamped()
            p1.header.frame_id = rospy.get_param("/end_effector_link")
            p1.pose.position = Point(x=0, y=0, z=0.5)
            try:
                p2 = self.tf_listener.transformPose("/fabrics_goal", p1)
                self.state.angular_error = np.linalg.norm(
                    np.array(list(p1.pose.position)) - list(p2.pose.position)
                )
            except Exception as e:
                rospy.loginfo(e)
                rospy.loginfo("Waiting for tf to be on time.")

            # update state
            self.state.goal_reached = (
                self.state.positional_error < self.positional_goal_tolerance
                and self.state.angular_error < self.angular_goal_tolerance
            )
        self.state.header.stamp = rospy.Time.now()
        self.state.tolerance_goal_0 = self.positional_goal_tolerance
        self.state.tolerance_goal_1 = self.angular_goal_tolerance
        self.state_publisher.publish(self.state)
        return self.state.goal_reached
