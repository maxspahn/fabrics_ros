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


class FabricsGoalEvaluator(object):
    def __init__(self):
        self.state = FabricsState(
            goal_reached=False, positional_error=1, angular_error=1
        )
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.load_parameters()
        rospy.sleep(1.0)
        self.state_publisher = rospy.Publisher("state", FabricsState, queue_size=10)

    def load_parameters(self):
        self.positional_goal_tolerance = rospy.get_param("/positional_goal_tolerance")
        self.angular_goal_tolerance = rospy.get_param("/angular_goal_tolerance")

    def evaluate(self, goal: FabricsGoal, joint_states: JointState) -> bool:
        if isinstance(goal, JointState):
            self.state.positional_error = np.linalg.norm(
                np.array(joint_states.position) - np.array(goal.position)
            )
            self.state.angular_error = 0.0
            self.state.goal_reached = (
                self.state.positional_error < self.positional_goal_tolerance
            )
        elif isinstance(goal, PoseStamped):
            # broadcast goal frame
            # Note: at 10x lower frequency than act loop, because tf
            if not hasattr(self, "eval_i"):
                self.eval_i = 0
            self.eval_i += 1
            if self.eval_i % 10 == 0:
                self.eval_i = 0
                self.tf_broadcaster.sendTransform(
                    list(goal.pose.position),
                    list(goal.pose.orientation),
                    rospy.Time.now(),
                    "/fabrics_goal",
                    "/panda_link0",
                )

            # evaluate positional_error
            trans, _ = self.tf_listener.lookupTransform(
                "/panda_link0", "/panda_vacuum", rospy.Time(0)
            )
            self.state.positional_error = np.linalg.norm(
                np.array(trans) - list(goal.pose.position)
            )

            # evaluate angular_error
            # Note: this is done by taking the L2 norm between a point at z=0.5 along the '/panda_link8' frame and transforming it to the goal frame
            p1 = PoseStamped()
            p1.header.frame_id = "/panda_vacuum"
            p1.pose.position = Point(x=0, y=0, z=0.5)
            try:
                p2 = self.tf_listener.transformPose("/fabrics_goal", p1)
                self.state.angular_error = np.linalg.norm(
                    np.array(list(p1.pose.position)) - list(p2.pose.position)
                )
            except Exception as _:
                rospy.loginfo("Waiting for tf to be on time.")

            # update state
            self.state.goal_reached = (
                self.state.positional_error < self.positional_goal_tolerance
                and self.state.angular_error < self.angular_goal_tolerance
            )
        self.state.header.stamp = rospy.Time.now()
        self.state_publisher.publish(self.state)
        return self.state.goal_reached
