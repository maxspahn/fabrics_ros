#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import time
import numpy as np
from kortex_driver.srv import *
from kortex_driver.msg import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class KinovaKortexNode:
    def __init__(self):
        rospy.init_node('kinova_kortex_node')
        self._frequency = 100
        self.rate = rospy.Rate(self._frequency)
        try:
            print("I am at init kinova kortex node")
            self.HOME_ACTION_IDENTIFIER = 2
            self.dof = rospy.get_param("degrees_of_freedom")
            self._q = np.zeros((self.dof, 1))
            self.fabrics_vel = np.zeros((self.dof, 1))
            self.fabrics_joints = None
            self.action_topic_sub = None
            self.all_notifs_succeeded = True

            self.all_notifs_succeeded = True

            # Get node params
            self.robot_name = "my_gen3" #rospy.get_param('~robot_name', "my_gen3")
            print("after self._robot_name")
            rospy.loginfo("Using robot_name " + self.robot_name)

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.joint_state_sub = rospy.Subscriber("/" + self.robot_name + "/joint_states", JointState, self.cb_joint_state)
            print("reached after joint state sub!!!")
            self.last_action_notif_type = None

            # Init the subscriber to fabrics: 
            self.fabrics_sub = rospy.Subscriber('/command_fabrics', Float64MultiArray,  self.cb_fabrics_vel)
            print("reached after fabrics_sub!!!")
            
            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            print("reached read action_full_name:", read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)
            print("reached after read action")
            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
            print("end of _init_   in kinova_kortex_node")
        except:
            print("in except init")
            self.is_init_success = False
        else:
            print("In else init")
            self.is_init_success = True
    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event
        
    def cb_fabrics_vel(self, msg):
        self.fabrics_msg = msg
        self.fabrics_vel = np.array(msg.data)
        self.integrate_vel_to_pos()
        
    def cb_joint_state(self, msg):
        self._q = np.array(msg.position)
        
    def integrate_vel_to_pos(self):
        delta_joint_position = self.fabrics_vel*rospy.get_param("/dt")*180/np.pi
        self.fabrics_joints = self._q +delta_joint_position
        # print("self.fabrics_joints: ", self.fabrics_joints)

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            else:
                time.sleep(0.01)

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")
            return True

        # Wait a bit
        rospy.sleep(0.25)

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)

        return True

    def main(self):
        ### initializations (not looped):
        success = self.is_init_success
        print("in Main loop of kinova kortex")
        
        try:
            rospy.delete_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python")
        except:
            pass
        
        
        # #*******************************************************************************
        # # Make sure to clear the robot's faults else it won't move if it's already in fault
        # success &= self.example_clear_faults()
        # #*******************************************************************************
                
        # #*******************************************************************************
        # # Start the example from the Home position
        # # success &= self.example_home_the_robot()
        # #*******************************************************************************

        # #*******************************************************************************
        # # Set the reference frame to "Mixed"
        # success &= self.example_set_cartesian_reference_frame()

        # #*******************************************************************************
        # # Subscribe to ActionNotification's from the robot to know when a cartesian pose is finished
        # success &= self.example_subscribe_to_a_robot_notification()
        
        # #*******************************************************************************
        req = ExecuteActionRequest()
                
        # Prepare and send cartesian pose ###
        my_cartesian_speed = CartesianSpeed()
        my_cartesian_speed.translation = 0.1 # m/s
        my_cartesian_speed.orientation = 15  # deg/s

        my_constrained_pose = ConstrainedPose()
        my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)

        my_constrained_pose.target_pose.x = 0.374
        my_constrained_pose.target_pose.y = 0.081
        my_constrained_pose.target_pose.z = 0.650
        my_constrained_pose.target_pose.theta_x = -57.6
        my_constrained_pose.target_pose.theta_y = 91.1
        my_constrained_pose.target_pose.theta_z = 2.3
        req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
        
        while not rospy.is_shutdown():
            t0 = time.perf_counter()

            if success == 1 or success == 0:
                
                # Make sure to clear the robot's faults else it won't move if it's already in fault
                #success &= self.example_clear_faults()
                
                        #*******************************************************************************
                # Make sure to clear the robot's faults else it won't move if it's already in fault
                # success &= self.example_clear_faults()
                # #*******************************************************************************
                        
                # #*******************************************************************************
                # # Start the example from the Home position
                # # success &= self.example_home_the_robot()
                # #*******************************************************************************

                # #*******************************************************************************
                # # Set the reference frame to "Mixed"
                # success &= self.example_set_cartesian_reference_frame()

                # # #*******************************************************************************
                # # # Subscribe to ActionNotification's from the robot to know when a cartesian pose is finished
                # success &= self.example_subscribe_to_a_robot_notification()
                
                #*******************************************************************************
                
                # prepare joint space goal
                my_constrained_joint_angles = ConstrainedJointAngles()
                # print("my_constrainted_joint_angles!!: ", my_constrained_joint_angles)
                joint_angles = []
                for i in range(self.dof): 
                    joint_angle = JointAngle()
                    print("joint_angle:", joint_angle)
                    joint_angle.joint_identifier = i
                    if self.fabrics_joints is None:
                        joint_angle.value = 0.
                    else:
                        # print("self.fabrics_joints: ", self.fabrics_joints)
                        joint_angle.value = 90. #self.fabrics_joints[i]*100000 #IN DEGREES!!!
                    joint_angles.append(joint_angle)
                print("joint_angles: ", joint_angles)
                my_constrained_joint_angles.joint_angles.joint_angles = joint_angles
                req.input.oneof_action_parameters.reach_joint_angles.append(my_constrained_joint_angles)
                # print("i AM AFTER JOINT ANGLES INPUT!!!")
                
                """
                #prepare joint velocity goal: this doesn't work, not supported!!!
                # Create a Base_JointSpeeds object
                joint_speeds_struct = Base_JointSpeeds()
                # Add speeds for each joint
                joint_speeds = []
                for i in range(self.dof):
                    joint_speed = JointSpeed()
                    joint_speed.joint_identifier = i
                    joint_speed.value = 0.0
                    joint_speed.duration = 1
                    joint_speeds.append(joint_speed)
                joint_speeds_struct.joint_speeds = joint_speeds
                # print("joint_speeds_struct: ", joint_speeds_struct)
                #req.input.oneof_action_parameters.send_joint_speeds = [joint_speeds_struct]
                """
                
                # give goal to req
                req.input.name = "pose1"
                req.input.handle.action_type = ActionType.REACH_JOINT_ANGLES #JOINT_ANGLES #REACH_POSE #SEND_JOINT_SPEEDS #REACH_JOINT_ANGLES 
                req.input.handle.identifier = 1001

                rospy.loginfo("Sending pose 1...")
                self.last_action_notif_type = None
                try:
                    print("I am at execute action")
                    # print("req: ", req)
                    self.execute_action(req)
                except rospy.ServiceException:
                    rospy.logerr("Failed to send pose 1")
                    success = False
                else:
                    print("in else statement waiting")
                    rospy.loginfo("Waiting for pose 1 to finish...")
                # print("req: ", req)
                # self.wait_for_action_end_or_abort()

                # success &= self.all_notifs_succeeded

                # success &= self.all_notifs_succeeded

            # For testing purposes
            print("Kinova kortex executed successfully!!")
            #rospy.set_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python", success)

            # if not success:
            #     rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = KinovaKortexNode()
    ex.main()