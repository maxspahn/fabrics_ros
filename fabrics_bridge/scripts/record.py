#!/usr/bin/env python3
import sys
import numpy as np
import rospy
from sensor_msgs.msg import JointState
import tf2_ros
import actionlib
import pickle
from std_msgs.msg import Bool, Float64MultiArray
from geometry_msgs.msg import PoseStamped, Vector3
from geometry_msgs.msg import Point, Quaternion
from albert_vacuum_gripper.msg import DropOffActionGoal, VacuumActionGoal

from forwardkinematics.urdfFks.generic_urdf_fk import GenericURDFFk

import panda_py as papy
from pynput import keyboard

# Helpers
def _it(self):
    yield self.x
    yield self.y
    yield self.z


Point.__iter__ = _it
Vector3.__iter__ = _it


def _it(self):
    yield self.x
    yield self.y
    yield self.z
    yield self.w


Quaternion.__iter__ = _it

class WrongInteractionDevice(Exception):
    pass

def moving_average(data, window_size):
    return np.convolve(data, np.ones((window_size, 3))/window_size, mode='valid')

class Recording():
    def __init__(self, ee_name, name):
        self._data = []
        self._ee_name = ee_name
        self._name = name
        self._indices = {
            'position_1': [0, 3],
            'position_2': [3, 6],
            'position_3': [6, 9],
            'header': 9,
            'weight_1': 10,
            'weight_2': 11,
            'weight_3': 12,
            'tolerance_1': 13,
            'tolerance_2': 14,
            'tolerance_3': 15,
            'vacuum': 16,
        }
        self._record_threshold = 0.0005

    def name(self, name: str) -> None:
        self._name = name

    def append_waypoint(self, waypoint: list) -> None:
        if len(self._data) < 1:
            distance = 1
        else:
            distance = np.linalg.norm(
                np.array(waypoint[0:3]) - np.array(self._data[-1][0:3])
            )
            vacuum_changed = self._data[-1][self._indices['vacuum']] == waypoint[self._indices['vacuum']]
        if (distance >= self._record_threshold) or vacuum_changed:
            self._data.append(waypoint)
        else:
            print("Waypoint discarded because it is too close to the old one.")

    def remove_waypoint(self, index: int) -> None:
        self._data.pop(index)

    def change_value_at_waypoint(self, index, data_name, new_value) -> None:
        try:
            self._data[index][self._indices[data_name]] = new_value
        except KeyError as _:
            print(f"Could not find data field with name {data_name}")

    def waypoints(self) -> list:
        return self._data


    def smoothen_trajectory(self):
        window_size = 20

        positions = np.array([d[0:3] for d in self._data])
        smooth_positions = np.zeros_like(positions)
        positions = np.pad(positions, [[window_size//2, window_size//2-1], [0, 0]], mode='edge')

        for i in range(3):
            smooth_positions[:, i] = np.convolve(positions[:, i], np.ones(window_size)/window_size, mode='valid')

        for i in range(len(self._data)):
            self._data[i][0:3] = smooth_positions[i, 0:3].tolist()

    def save(self, smoothen: bool = False):
        if smoothen:
            self.smoothen_trajectory()
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
        rospy.sleep(1.0)
        self._time = rospy.Time.now()
        rospy.loginfo("Starting RecordSkillNode as record_skill.")

        urdf = rospy.get_param(rospy.get_param("/urdf_source"))
        self._index_list = rospy.get_param("/index_list")
        self._fk = GenericURDFFk(
            urdf,
            rootLink=rospy.get_param("/root_link"),
            end_link=rospy.get_param("/end_links"),
        )
        self.establish_ros_connections()
        self.set_controller()

        self._weight_factor = 0.5
        self.joint_states = JointState()
        self._skill_name = skill_name
        self._vacuum = False
        self._ee_frame = ee_frame
        self._target_frame = target_frame
        self.use_static_frame = True

        self._currently_recording = False
        self.set_interaction_device(interaction_device)
            
        self._record_counter = 0
        

    def establish_ros_connections(self):
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
        self.joint_state_subscriber = rospy.Subscriber(
            "/joint_states_filtered",
            JointState,
            self.joint_states_callback,
            tcp_nodelay=True,
        )
        self._suck_publisher = rospy.Publisher("/franka_vacuum_gripper/vacuum/goal", VacuumActionGoal, queue_size=1)
        self._unsuck_publisher = rospy.Publisher("/franka_vacuum_gripper/dropoff/goal", DropOffActionGoal, queue_size=1)

    def joint_states_callback(self, msg: JointState):
        # TODO: better way of extracting just the panda joint states
        self.joint_states.name = np.array(msg.name)[self._index_list]
        self.joint_states.position = np.array(msg.position)[self._index_list]
        self.joint_states.velocity = np.array(msg.velocity)[self._index_list]

    def suck(self):
        goal_msg = VacuumActionGoal()
        goal_msg.goal.vacuum = 1
        self._suck_publisher.publish(goal_msg)

    def unsuck(self):
        goal_msg = DropOffActionGoal()
        self._unsuck_publisher.publish(goal_msg)

    def set_controller(self):
        ki = [0.0, ] * 7
        kp = [0.1, ] * 7
        ki_msg = Float64MultiArray(data=ki)
        kp_msg = Float64MultiArray(data=kp)
        for i in range(20):
            rospy.loginfo("Setting low controller gains.")
            self._ki_publisher.publish(ki_msg)
            self._kp_publisher.publish(kp_msg)
            rospy.sleep(0.1)

    def set_interaction_device(self, interaction_device):
        if interaction_device == 'desk':
            self._desk = papy.Desk('172.16.0.2', 'admin', '0123456789')
            self._desk.listen(self.panda_button_cb)
        elif interaction_device == 'keyboard':
            rospy.loginfo("Press r to start recording and e to end recording")
            rospy.loginfo("Press v and n to activate and deactive the suction.")
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
        self._time = rospy.Time.now()
        if not hasattr(self, '_recording'):
            self._record_counter += 1
            skill_name = self._skill_name + "_" + str(self._record_counter)
            self._recording = Recording(self._ee_frame, skill_name)
            rospy.loginfo(f"Recording skill {skill_name}")

    def stop_recording(self):
            self._currently_recording = False
            if hasattr(self, '_recording'):
                self.save_poses()
                self.unsuck()

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
        if event == keyboard.KeyCode.from_char("v"):
            self.suck()
            self._vacuum = True
        if event == keyboard.KeyCode.from_char("n"):
            self.unsuck()
            self._vacuum = False

            
            


    def save_poses(self):
        self._recording.save(smoothen=True)
        del self._recording

    def run(self):
        r = rospy.Rate(20)
        rospy.sleep(1)
        weight_0 = 1.0
        weight_1 = 6.0
        threshold_0 = 0.03



        """
        if self.use_static_frame:
            self.broadcast_static_frame()
            lookup_reference_frame = "recording"
        else:
            lookup_reference_frame = self._target_frame
        """

        while not rospy.is_shutdown():
            """
            try:
                trans = self.tf_buffer.lookup_transform(
                    lookup_reference_frame, self._ee_frame, rospy.Time()
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
            pose.header.frame_id = self._target_frame
            pose.pose.position = trans.transform.translation
            pose.pose.orientation = trans.transform.rotation
            """

            q = np.array(self.joint_states.position)
            position_1_child = self._fk.fk(q, 'panda_link0', 'panda_vacuum1_link', positionOnly=True)
            position_2_parent = self._fk.fk(q, 'panda_link0', 'panda_link7', positionOnly=True)
            position_2_child = self._fk.fk(q, 'panda_link0', 'panda_vacuum_support_link', positionOnly=True)
            position_3_parent = self._fk.fk(q, 'panda_link0', 'panda_vacuum1_link', positionOnly=True)
            position_3_child = self._fk.fk(q, 'panda_link0', 'panda_vacuum2_link', positionOnly=True)

            position_1 = position_1_child
            position_2 = position_2_child - position_2_parent
            position_3 = position_3_child - position_3_parent
            weight_1 = self._weight_factor / np.linalg.norm(position_1)
            weight_2 = self._weight_factor * 1.5 / np.linalg.norm(position_2)
            weight_3 = self._weight_factor * 1.5 / np.linalg.norm(position_3)
            threshold_1 = 0.3
            threshold_2 = 0.3
            threshold_3 = 0.3
            duration_since_start = rospy.Time.now()-self._time

            # Add the pose to the list
            if self._currently_recording:
                self._recording.append_waypoint(
                    position_1.tolist() \
                    + position_2.tolist() \
                    + position_3.tolist() \
                    + [
                        int(duration_since_start.to_sec()),
                        weight_1,
                        weight_2,
                        weight_3,
                        threshold_1,
                        threshold_2,
                        threshold_3,
                        self._vacuum,
                    ]
                )
                #rospy.loginfo("Currently Recording")

            else:
                a = 3
                #rospy.loginfo("Not recording. Press circle on panda to start, cross to stop.")
            r.sleep()

    def broadcast_static_frame(self):
        while True:
            try:
                static_playback_frame_transform = self.tf_buffer.lookup_transform('panda_link0', self._target_frame, rospy.Time(0))
                break
            except Exception as e:
                rospy.logwarn(f"could not get transform for static playback frame, see error {e}")
        static_playback_frame_transform.child_frame_id = 'recording'
        self.tf_broadcaster.sendTransform(static_playback_frame_transform)
        rospy.sleep(0.2)


if __name__ == "__main__":
    record_skill = RecordSkillNode(
        skill_name=sys.argv[2],
        target_frame="desired_product",
        ee_frame="panda_vacuum",
        interaction_device=sys.argv[1],
    )
    record_skill.run()
