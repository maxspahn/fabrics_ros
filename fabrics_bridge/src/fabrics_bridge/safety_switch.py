#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from kortex_driver.msg import *

class SafetySwitchNode():
    def __init__(self):
        rospy.init_node("safety_switch_node")
        self._dt = 0.05
        self.msg = None
        self.vel_topic = '/arm/in/joint_velocity' #'/joints_velocity_controller/command'
        self.joystick_topic = '/bluetooth_teleop/joy'
        print("I reached here!!!")
        self.establish_ros_connections()
        self._rate = rospy.Rate(1/self._dt)
        print("I reached here2!!!")

        self.keep_output_enabled_for = 0

    def establish_ros_connections(self):

        self._vel_pub = rospy.Publisher(self.vel_topic, Base_JointSpeeds, queue_size=1)

        self.joystick_sub = rospy.Subscriber(self.joystick_topic, Joy, self.joystick_cb, queue_size=1)
        
        self.action_subscriber = rospy.Subscriber(
            "/command_fabrics",
            Base_JointSpeeds,
            self.action_fabrics_callback,
            tcp_nodelay=True,
        )
        
    def action_fabrics_callback(self, msg: Base_JointSpeeds):
        self.msg = msg

    def joystick_cb(self, msg):
        # This is used as a safety command for the real robot
        if msg.axes[2] <= -0.9: # press L2 on ps4 controller
            self.enable_output = True
            self.keep_output_enabled_for = 2 # budget of two to prevent race condition
            print("I am here at enable output!!")
        else:
            self.keep_output_enabled_for = 0

    def actuate_if_deadman_switch(self):
        if self.keep_output_enabled_for > 0:
            self.enable_output = True
            self.keep_output_enabled_for -= 1
            # Publish output command
            self._vel_pub.publish(self.vel_cmd_msg)
            #print("publishing output")
        else:
            if self.vel_cmd_msg is not None:
                for joint_speed in self.vel_cmd_msg.joint_speeds:
                    joint_speed.value = 0.
                # self.vel_cmd_msg.data = [0., 0., 0., 0., 0., 0.] #todo: remove hardcoded!
                self._vel_pub.publish(self.vel_cmd_msg)
                #print("I am here2222")
            self.enable_output = False

    def act(self):
        self.vel_cmd_msg = Base_JointSpeeds()
        self.vel_cmd_msg = self.msg

        self.actuate_if_deadman_switch()
        #print("self.msg:", self.msg)

    def run(self):
        while not rospy.is_shutdown():
            self.act()
            self._rate.sleep()

if __name__ == "__main__":
    try:
        safety_switch_node = SafetySwitchNode()
        safety_switch_node.run()
    except rospy.ROSInterruptException:
        pass