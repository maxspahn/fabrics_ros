#!/usr/bin/env python3
import sys
import numpy as np
import rospy
import tf2_ros
import actionlib
import pickle
from std_msgs.msg import Bool, Float64MultiArray
from geometry_msgs.msg import PoseStamped

import panda_py as papy
from pynput import keyboard

class WrongInteractionDevice(Exception):
    pass

class Recording():
    def __init__(self, ee_name, name):
        self._data = []
        self._ee_name = ee_name
        self._name = name

    def name(self, name: str) -> None:
        self._name = name

    def append_waypoint(self, waypoint: list) -> None:
        self._data.append(waypoint)

    def remove_waypoint(self, index: int) -> None:
        self._data.pop(index)

    def change_weight_at_waypoint(self, index, new_weight) -> None:
        self._data[index][1] = new_weight

    def waypoints(self) -> list:
        return self._data

    def save(self):
        with open(f"{self._name}.pickle", "wb") as f:
            pickle.dump(self, f)
        rospy.loginfo(f"Saved {len(self._data)} poses to {self._name}.pickle")

    def __str__(self) -> str:
        return str(np.array(self._data)[:, 1])




class RecordSkillNode:
    def __init__(
            self,
            skill_name: str,
            target_frame: str,
            ee_frame: str,
            interaction_device: str
        ):
        rospy.init_node("record_skill")
        self.establish_ros_connections()
        rospy.sleep(1.0)
        rospy.loginfo("Starting RecordSkillNode as record_skill.")

        self._skill_name = skill_name
        self._ee_frame = ee_frame
        self._target_frame = target_frame

        self._currently_recording = False
        self.set_interaction_device(interaction_device)
            
        self._record_counter = 0
        self.set_controller()
        

    def establish_ros_connections(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self._ki_publisher = rospy.Publisher(
            "/compliant_joint_velocity_controller/controller_ki",
            Float64MultiArray,
            queue_size=1
        )
        self._kp_publisher = rospy.Publisher(
            "/compliant_joint_velocity_controller/controller_kp",
            Float64MultiArray,
            queue_size=1
        )

    def set_controller(self):
        ki = [0.0, ] * 7
        kp = [0.1, ] * 7
        ki_msg = Float64MultiArray(data=ki)
        kp_msg = Float64MultiArray(data=kp)
        rospy.loginfo("Setting low controller gains.")
        for i in range(10):
            self._ki_publisher.publish(ki_msg)
            self._kp_publisher.publish(kp_msg)

    def set_interaction_device(self, interaction_device):
        if interaction_device == 'desk':
            self._desk = papy.Desk('172.16.0.2', 'admin', '0123456789')
            self._desk.listen(self.panda_button_cb)
        elif interaction_device == 'keyboard':
            rospy.loginfo("Press r to start recording and e to end recording")
            self._listener = keyboard.Listener(
                    on_press=self.keyboard_cb,
            )
            self._listener.start()
        elif interaction_device == "tablet":
            self.start_tablet_subscribers()
        else:
            raise WrongInteractionDevice(
                    "Device %s is not available. " % interaction_device + 
                    "Use one of [desk, tablet, keyboard]")


    def start_tablet_subscribers(self):
        self._start_record_sub = rospy.Subscriber("/tablet/start_record", Bool, self.start_record_callback)
        self._stop_record_sub = rospy.Subscriber("/tablet/stop_record", Bool, self.stop_record_callback)

    def start_record_callback(self, msg):
        if msg.data:
            self.start_recording()

    def stop_record_callback(self, msg):
        if msg.data:
            self.stop_recording()

    def start_recording(self):
        self._currently_recording = True
        if not hasattr(self, '_recording'):
            self._record_counter += 1
            skill_name = self._skill_name + "_" + str(self._record_counter)
            self._recording = Recording(self._ee_frame, skill_name)
            rospy.loginfo(f"Recording skill {skill_name}")

    def stop_recording(self):
            self._currently_recording = False
            if hasattr(self, '_recording'):
                self.save_poses()

    def panda_button_cb(self, event):
        print(event)
        if 'circle' in event and event['circle'] == True :
            self.start_recording()
        if 'cross' in event and event['cross'] == True:
            self.stop_recording()

    def keyboard_cb(self, event):
        if event == keyboard.KeyCode.from_char("r"):
            self.start_recording()
        if event == keyboard.KeyCode.from_char("e"):
            self.stop_recording()

            
            


    def save_poses(self):
        self._recording.save()
        del self._recording

    def run(self):
        r = rospy.Rate(20)
        rospy.sleep(1)
        weight_0 = 1.0
        weight_1 = 1.0

        while not rospy.is_shutdown():
            try:
                trans = self.tf_buffer.lookup_transform(
                    self._target_frame, self._ee_frame, rospy.Time()
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                rospy.logwarn("Could not get transform")
                continue

            # Convert the transform to a PoseStamped message and add it to the list
            pose = PoseStamped()
            pose.header = trans.header
            pose.pose.position = trans.transform.translation
            pose.pose.orientation = trans.transform.rotation

            # Add the pose to the list
            if self._currently_recording:
                self._recording.append_waypoint([pose, weight_0, weight_1])
                rospy.loginfo("Currently Recording")

            else:
                a = 3
                #rospy.loginfo("Not recording. Press circle on panda to start, cross to stop.")
            r.sleep()


if __name__ == "__main__":
    record_skill = RecordSkillNode(
        skill_name=sys.argv[2],
        target_frame="panda_link0",
        ee_frame="panda_vacuum",
        interaction_device=sys.argv[1],
    )
    record_skill.run()
