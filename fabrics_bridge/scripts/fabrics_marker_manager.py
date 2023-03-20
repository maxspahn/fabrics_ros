#!/usr/bin/env python3
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import rospy
from fabrics_msgs.msg import (
    FabricsGoal,
)
def init_marker(a, r, g, b, frame_id, radius, identifier, type=2) -> Marker:
    marker = Marker()
    marker.header.frame_id = rospy.get_param("/root_link")
    marker.pose.orientation.w = 1.0
    marker.type = type
    marker.action = marker.ADD
    marker.color = ColorRGBA(a=a, r=r, g=g, b=b)
    marker.header.frame_id = frame_id
    diameter_body = 2 * radius
    marker.scale = Vector3(diameter_body,diameter_body,diameter_body) 
    marker.id = identifier
    return marker





class FabricsMarkerManager(object):
    def __init__(self, num_obstacles: int, collision_bodies: dict, collision_links: list, self_collision_pairs: dict):
        self.num_obstacles = num_obstacles
        self.collision_bodies = collision_bodies
        self.collision_links = collision_links
        self.self_collision_pairs = self_collision_pairs
        self.init_markers()
        self.init_collision_markers()
        self.init_self_collision_markers()
        self.init_publishers()

    def init_publishers(self):
        self.goal_marker_publisher = rospy.Publisher(
            "planning_goal/marker", Marker, queue_size=10
        )
        self.obs_markers_publisher = rospy.Publisher(
            "planning_obs/markers", MarkerArray, queue_size=10
        )
        self.collision_links_markers_publisher = rospy.Publisher(
            "planning_links/markers", MarkerArray, queue_size=10
        )
        self.self_collision_markers_publisher = {}
        for self_collision_link in self.self_collision_pairs.keys():
            name = f"planning_self_collision_link_{self_collision_link}"
            self.self_collision_markers_publisher[self_collision_link] = rospy.Publisher(
                name, MarkerArray, queue_size=10
            )


    def init_markers(self):

        self.goal_marker = init_marker(1, 0, 0, 1, rospy.get_param("/root_link"), 0.0, 1, type=0)
        self.goal_marker.points = [Point(x=0, y=0, z=0), Point(x=0, y=0, z=0.35)]
        self.goal_marker.id = 1
        self.goal_marker.scale = Vector3(0.02, 0.04, 0.04)

        self.obs_markers = MarkerArray()
        self.obs_markers.markers = [
            init_marker(1, 1, 0, 0, rospy.get_param("/root_link"), 0.0, i) for i in range(self.num_obstacles)
        ]

    def init_collision_markers(self):
        self.collision_link_markers = MarkerArray()
        for i, collision_link in enumerate(self.collision_links):
            radius_body = self.collision_bodies[f"radius_body_{collision_link}"] 
            marker = init_marker(1, 1, 0, 0, collision_link, radius_body, i)
            self.collision_link_markers.markers.append(marker)

    def init_self_collision_markers(self):
        self.self_collision_link_markers = {}
        for self_collision_link, paired_links in self.self_collision_pairs.items():
            marker_array = MarkerArray()
            radius_body = self.collision_bodies[f"radius_body_{self_collision_link}"]
            marker = init_marker(1, 0, 1, 0, self_collision_link, radius_body, 1)
            marker.header.frame_id = self_collision_link
            marker_array.markers.append(marker)
            for i, collision_link in enumerate(paired_links):
                radius_body = self.collision_bodies[f"radius_body_{collision_link}"]
                marker = init_marker(1, 0, 0, 1, collision_link, radius_body, i+2)
                marker_array.markers.append(marker)
            self.self_collision_link_markers[self_collision_link] = marker_array


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
            self.obs_markers.markers[i].scale = Vector3(1, 1, 1)
            if i < len(obstacles):
                o = obstacles[i]
                self.obs_markers.markers[i].pose.orientation.w = 1.0
                self.obs_markers.markers[i].pose.position = o.position
                self.obs_markers.markers[i].scale = Vector3(
                    o.radius * 2, o.radius * 2, o.radius * 2
                )
                self.obs_markers.markers[i].type = 2
                self.obs_markers.markers[i].color = ColorRGBA(a=1, r=1, g=0, b=0)
            else:
                self.obs_markers.markers[i] = init_marker(0.01, 0, 0, 1, rospy.get_param("/root_link"), 0.01, i)
        self.obs_markers_publisher.publish(self.obs_markers)
        self.collision_links_markers_publisher.publish(self.collision_link_markers)
        for self_collision_link in self.self_collision_pairs.keys():
            self.self_collision_markers_publisher[self_collision_link].publish(self.self_collision_link_markers[self_collision_link])
