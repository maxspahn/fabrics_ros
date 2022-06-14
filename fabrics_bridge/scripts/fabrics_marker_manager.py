#!/usr/bin/env python3
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class FabricsMarkerManager(object):
    def __init__(self, num_obstacles: int):
        self.num_obstacles = num_obstacles
        self.init_markers()

    def init_markers(self):
        def init_marker(a, r, g, b, type=2):
            goal_marker = Marker()
            goal_marker.header.frame_id = "panda_link0"
            goal_marker.type = type
            goal_marker.action = goal_marker.ADD
            goal_marker.color = ColorRGBA(a=a, r=r, g=g, b=b)
            return goal_marker

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

    def update_markers(self, goal, obstacles, goal_is_reached: bool):
        if isinstance(goal, PoseStamped):
            self.goal_marker.color.g = 1.0 if goal_is_reached else 0.0
            self.goal_marker.color.b = 0.0 if goal_is_reached else 1.0

            self.goal_marker.pose = goal.pose

            for i, o in enumerate(obstacles):
                if i >= self.num_obstacles:
                    break
                self.obs_markers.markers[i].pose.orientation = Quaternion(
                    x=0, y=0, z=0, w=1
                )
                self.obs_markers.markers[i].pose.position = o.position
                self.obs_markers.markers[i].scale = Vector3(
                    o.radius * 2, o.radius * 2, o.radius * 2
                )
