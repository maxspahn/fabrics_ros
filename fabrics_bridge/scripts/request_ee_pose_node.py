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
)

# Helper functions for making Point and Quaternion iterable
# def _it_point(self):
#     yield self.x
#     yield self.y
#     yield self.z


# Point.__iter__ = _it_point


# def _it_quat(self):
#     yield self.x
#     yield self.y
#     yield self.z
#     yield self.w


# Quaternion.__iter__ = _it_quat


class ClientNode(object):
    def __init__(self):
        rospy.init_node("client_node")
        self._rate = rospy.Rate(10)
        # default number of obstacles is 20
        self._num_obs = 1
        self.init_publishers()
        # ring of spheres setting
        # radius of shelf (suppose it's a circular shelf)
        self.r_shelf = 0.30
        # diameter of a sphere
        self.r_obs = 0.15
        self.obs = FabricsObstacleArray()
        self._obstacles = [[2.0, 2.0, 0.0]]
        self.dim_state = rospy.get_param("/dim_state")
        self.init_goal()

    def init_publishers(self):
        self._goal_publisher = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )
                # rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.cb_goal_rviz,
                #          tcp_nodelay=True,)
        # self._obs_publisher = rospy.Publisher(
        #     "fabrics/planning_obs", FabricsObstacleArray, queue_size=10
        # )

    # def append_obstacles(self, ring_obs):
    #     for obst in self._obstacles:
    #         o = FabricsObstacle()
    #         o.header.frame_id = rospy.get_param("/root_link")
    #         o.radius = self.r_obs
    #         o.position = Point(x=obst[0], y=obst[1], z=obst[2])
    #         o.obstacle_type = "sphere"
    #         self.obs.obstacles.append(o)
    #     return self.obs

    def publish_goal(self):
        self._goal_publisher.publish(self._goal)

    # def init_goal(self):
    #     self._goal = FabricsConstraintsGoal()
    #     self._goal.constraints = []
    #     goal_1 = FabricsConstraint()
    #     goal_1.weight = 1.0
    #     goal_1.indices = list(range(self.dim_state))
    #     goal_1.tolerance = 0.04
    #     goal_1.geometric_constraint = Float64MultiArray(data=[-0.0, -0.0])
    #     goal_1.parent_link = rospy.get_param("/root_link")
    #     goal_1.child_link = rospy.get_param("/end_effector_link")
    #     self._goal.constraints.append(goal_1)

    # def publish_obs(self):
    #     self.obs.obstacles = []
    #     self.append_obstacles(self._obstacles)
    #     self._obs_publisher.publish(self.obs)

    def init_goal(self):
        self._goal = PoseStamped()
        self._goal.pose.position.x = 0.5
        self._goal.pose.position.y = 0.5
        self._goal.pose.position.z = 0.5 
        return 
        # if self.dim_state == 2:
        #     goal_position = [self._goal.pose.position.x, self._goal.pose.position.y]
        # elif self.dim_state == 3:
        #     goal_position = [self._goal.pose.position.x, self._goal.pose.position.y, self._goal.pose.position.z]
        # else:
        #     print("This goal-space dimension is not known!")
        # print("goal position: ", goal_position)
        # goal_dict = {
        #     "subgoal0": {
        #         "weight": rospy.get_param("/weight_goal0"),
        #         "is_primary_goal": True,
        #         "indices": list(range(self.dim_state)),
        #         "parent_link" : rospy.get_param("/root_link"),
        #         "child_link" : rospy.get_param("/end_effector_link"),
        #         "desired_position": goal_position,
        #         "epsilon" : 0.1,
        #         "type": "staticSubGoal"
        #     }
        # }
        # goal = GoalComposition(name="goal", content_dict=goal_dict)
        # return goal

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
