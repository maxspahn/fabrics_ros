#!/usr/bin/env python3
# general python imports
import numpy as np
import math
import time
import tf
from scipy.spatial.transform import Rotation as R

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
    def __init__(self, WITH_ORIENTATION=True):
        rospy.init_node("client_node")
        self.WITH_ORIENTATION = WITH_ORIENTATION
        self._rate = rospy.Rate(500)
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
        self.x_goal = [0.5, -0.5, 0.3]
        # self.x_goal = rospy.get_param("/x_goal0")
        self._goal = PoseStamped()
        
        self._goal.pose.position.x = self.x_goal[0]
        self._goal.pose.position.y = self.x_goal[1]
        self._goal.pose.position.z = self.x_goal[2]
        return 
    
    def init_goal_pose(self):
        self.x_goal = [0.5, -0.5, 0.3]
        self.orientation_euler = np.array([0, 90, 0]) * np.pi / 180
        r = R.from_euler("zyx", self.orientation_euler)
        self.orientation_quat = [0, 0, 0, 1] #r.as_quat()
        print("self.orientation_quat: ", self.orientation_quat)
        
        self._goal = FabricsPoseGoal()
        self._goal.goal_pose.position.x = self.x_goal[0]
        self._goal.goal_pose.position.y = self.x_goal[1]
        self._goal.goal_pose.position.z = self.x_goal[2]
        self._goal.weight_position = 50
        self._goal.goal_pose.orientation.x = self.orientation_quat[0]
        self._goal.goal_pose.orientation.y = self.orientation_quat[1]
        self._goal.goal_pose.orientation.z = self.orientation_quat[2]
        self._goal.goal_pose.orientation.w = self.orientation_quat[3]
        self._goal.weight_orientation = 50
        
        print("self._goal: ", self._goal)
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
