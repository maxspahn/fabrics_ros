#!/usr/bin/env python3
# general python imports
import numpy as np
import math
import time
import tf

# ros imports
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from fabrics_msgs.msg import (
    FabricsConstraintsGoal,
    FabricsConstraint,
    FabricsObstacle,
    FabricsObstacleArray,
    FabricsPoseGoal
)

class ClientNode(object):
    def __init__(self, WITH_ORIENTATION=False):
        rospy.init_node("client_node")
        self.WITH_ORIENTATION = WITH_ORIENTATION
        self._rate = rospy.Rate(10)
        self._num_obs = 1
        self.init_publishers()
        self.dim_state = rospy.get_param("/dim_state")
        if WITH_ORIENTATION == True:
            self.init_goal_pose()
        else:
            self.init_goal_position()

    def init_publishers(self):
        if self.WITH_ORIENTATION == True:
            self._goal_publisher = rospy.Publisher(
                "fabrics/ee_pose_goal", FabricsPoseGoal, queue_size=10
            )
        else:
            self._goal_publisher = rospy.Publisher(
                "/move_base_simple/goal", PoseStamped, queue_size=10
            )

    def publish_goal(self):
        self._goal_publisher.publish(self._goal)

    def init_goal_position(self):
        self._goal = PoseStamped()
        self._goal.pose.position.x = 0.737
        self._goal.pose.position.y = 0.163
        self._goal.pose.position.z = 0.461
        return 
    
    def init_goal_pose(self):
        self._goal = FabricsPoseGoal()
        self._goal.goal_pose.position.x = 0.737
        self._goal.goal_pose.position.y = 0.163
        self._goal.goal_pose.position.z = 0.461
        
        self._goal.goal_pose.orientation.x = 0
        self._goal.goal_pose.orientation.y = 0
        self._goal.goal_pose.orientation.z = 0
        self._goal.goal_pose.orientation.w = 1
        return

    def run(self):
        while not rospy.is_shutdown():
            self.publish_goal()
            self._rate.sleep()


if __name__ == "__main__":
    client_node = ClientNode()
    try:
        client_node.run()
    except rospy.ROSInterruptException:
        pass
