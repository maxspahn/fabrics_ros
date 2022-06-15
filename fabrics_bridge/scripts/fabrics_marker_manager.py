#!/usr/bin/env python3
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import rospy
from fabrics_msgs.msg import (
    FabricsGoal,
)
def init_marker(a, r, g, b, type=2):
    goal_marker = Marker()
    goal_marker.header.frame_id = "panda_link0"
    goal_marker.type = type
    goal_marker.action = goal_marker.ADD
    goal_marker.color = ColorRGBA(a=a, r=r, g=g, b=b)
    return goal_marker


class FabricsMarkerManager(object):
    def __init__(self, num_obstacles: int):
        self.num_obstacles = num_obstacles
        self.init_markers()
        self.init_publishers()

    def init_publishers(self):
        self.goal_marker_publisher = rospy.Publisher(
            "planning_goal/marker", Marker, queue_size=10
        )
        self.obs_markers_publisher = rospy.Publisher(
            "planning_obs/markers", MarkerArray, queue_size=10
        )

    def init_markers(self):

        self.goal_marker = init_marker(1, 0, 0, 1, type=0)
        self.goal_marker.points = [Point(x=0, y=0, z=0), Point(x=0, y=0, z=0.35)]
        self.goal_marker.id = 1
        self.goal_marker.scale = Vector3(0.02, 0.04, 0.04)

        self.obs_markers = MarkerArray()
        self.obs_markers.markers = [
            init_marker(1, 1, 0, 0) for _ in range(self.num_obstacles)
        ]
        for i, m in enumerate(self.obs_markers.markers):
            m.id = i

    def update_markers(self, goal: FabricsGoal, obstacles, goal_is_reached: bool):
        if goal.goal_type == 'ee_pose':
            self.goal_marker.color.g = 1.0 if goal_is_reached else 0.0
            self.goal_marker.color.b = 0.0 if goal_is_reached else 1.0

            self.goal_marker.pose = goal.goal_pose.pose
            self.goal_marker_publisher.publish(self.goal_marker)

        for i in range(self.num_obstacles):
            self.obs_markers.markers[i].pose.orientation = Quaternion(
                x=0, y=0, z=0, w=1
            )
            if i < len(obstacles):
                o = obstacles[i]
                self.obs_markers.markers[i].pose.position = o.position
                self.obs_markers.markers[i].scale = Vector3(
                    o.radius * 2, o.radius * 2, o.radius * 2
                )
                self.obs_markers.markers[i].type = 2
                self.obs_markers.markers[i].color = ColorRGBA(a=1, r=1, g=0, b=0)
            else:
                self.obs_markers.markers[i] = init_marker(1, 0, 0, 1, type=0)
                self.obs_markers.markers[i].id = i
        self.obs_markers_publisher.publish(self.obs_markers)
