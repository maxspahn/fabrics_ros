#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from pykalman import KalmanFilter
import copy


class JointStatesKF(object):

    def __init__(self):

        # config
        self._rate = 100
        self._ros_rate = rospy.Rate(self._rate)
        self._dt = 1./self._rate

        # Publisher of filtered joint states
        self._filter_joint_states_pub = rospy.Publisher(
            '/joint_states_filtered',
            JointState, queue_size=10
        )

        # Publisher desired torque commands to joints
        self._torque_command_pub = rospy.Publisher(
            '/panda_joint_effort_controller/command',
            Float64MultiArray, queue_size=10
        )

        # Create combined JointState Listener listener
        self._joint_states_listener = AlbertJointStates()

        # Hack to simulate acceleration observation by single step integration
        self._acceleration_estimate = np.array([0]*len(self._joint_states_listener.current.name))

        self._init_kf()

        # Final filtered joint states to publish
        self._joint_states_filtered = JointState()
        self._joint_states_filtered.name = self._joint_states_listener.current.name

        # Hack to simulate acceleration commands
        self._acceleration_command = [0]*7
        self.prev_err = [0]*7
        self.prev_err_i = [0]*7
        self.torque_command = Float64MultiArray()


    def _init_kf(self):
        """
        Combined KalmanFilter for each joint. There should be no covariance between joints, since they are independent.
        But I've combined it to make the computation more efficient (matrix multiplies).
        """
        self.num_joints = len(self._joint_states_listener.current.name)
        transition_matrix = np.eye(3*self.num_joints)
        transition_matrix[0:self.num_joints, self.num_joints:2*self.num_joints] = np.eye(self.num_joints)*self._dt

        # Oberservation model. Can observe pos and vel and acc
        observation_matrix = np.eye(3*self.num_joints)

        # Covariances. Can be tuned to improve tracking / reduce noise of state estimates
        transition_covariance = np.eye(3*self.num_joints)*np.repeat([0.01, 0.01, 0.01], self.num_joints)
        observation_covariance = np.eye(3*self.num_joints)*np.repeat([0.1, 5.5, 16.5], self.num_joints)

        self._filtered_state_means = [0]*3*self.num_joints
        self._filtered_state_covariances = np.eye(3*self.num_joints)*0.5
        self._filtered_state = JointState()

        self._kf = KalmanFilter(
            transition_matrices=transition_matrix,
            observation_matrices=observation_matrix,
            transition_covariance=transition_covariance,
            observation_covariance=observation_covariance,
        )

    def pub_filtered_states(self):
        # self._joint_states_filtered.position     = list(self._filtered_state_means[:self.num_joints])
        self._joint_states_filtered.velocity     = list(self._filtered_state_means[self.num_joints:2*self.num_joints])
        self._joint_states_filtered.effort = list(self._filtered_state_means[2*self.num_joints:])
        self._joint_states_filtered.position     = list(self._acceleration_estimate)
        # self._joint_states_filtered.acceleration = list(self._filtered_state_means[2*self.num_joints:])
        self._filter_joint_states_pub.publish(self._joint_states_filtered)

    def run(self):
        self._ros_rate.sleep()
        while not rospy.is_shutdown():

            # Update state estimates
            current_joint_state = self._joint_states_listener.current
            observations = current_joint_state.position + \
                           current_joint_state.velocity + \
                           list(self._acceleration_estimate)
            vel_old = self._filtered_state_means[self.num_joints:2*self.num_joints]
            self._filtered_state_means, self._filtered_state_covariances = (
                self._kf.filter_update(
                    self._filtered_state_means,
                    self._filtered_state_covariances,
                    observations
                )
            )
            vel_new = np.array(self._filtered_state_means[self.num_joints:2*self.num_joints])
            self._acceleration_estimate = (vel_new - vel_old)/self._dt
            self.pub_filtered_states()

            # Control acceleration (PID)
            # k_p = np.array([100., 50., 20., 10., 5., 1., 0.5])*0.01
            k_p = np.array([0., 20.0, 0.0, 0.0, 0.0, 0.0, 0.05])
            k_i = [0.00,  0.5, 0., 0., 0., 0., 0.00]
            k_d = [0.00,  0.1, 0., 0., 0., 0., 0.000]
            print(self._acceleration_command)
            err = self._acceleration_command - np.array(self._acceleration_estimate[5:])
            err_i = self.prev_err_i + err*self._dt
            err_d = (err-self.prev_err)/self._dt
            command = k_p*err + k_i*err_i + k_d*err_d
            self.torque_command.data = command #np.where(abs(command) > 2, 0, command)
            # self.torque_command.data = np.clip(k_p*err + k_i*err_i + k_d*err_d, -2, 2)
            
            # test = k_p*err + k_i*err_i + k_d*err_d
            # self.torque_command.data = [0]*7
            # self.torque_command.data[0] = test[0]
            self.prev_err = err
            self.prev_err_i = err_i
            print(self._acceleration_estimate[5:])
            print(err)
            print(self.torque_command.data)
            print(" ")
            self._torque_command_pub.publish(self.torque_command)

            self._ros_rate.sleep()

class AlbertJointStates(object):
    def __init__(self):
        self._init_subs()

        self._joint_state = JointState()
        self._joint_state.name = [
            'boxer_x',
            'boxer_y',
            'boxer_theta',
            'vel_forward',
            'vel_rotational'] + ['panda_joint{}'.format(i+1) for i in range(7)]
        self._joint_state.position = [0]*len(self._joint_state.name)
        self._joint_state.velocity = [0]*len(self._joint_state.name)


    def _init_subs(self):
        # Subscriber to the joint states topic
        self._joint_states_sub = rospy.Subscriber(
            '/joint_states',
            JointState, self.joint_states_cb,
            tcp_nodelay=True
        )

        # Subscriber to the robot odometry
        self._odometry_sub = rospy.Subscriber(
            '/odometry/filtered',
            Odometry, self.odometry_cb,
            tcp_nodelay=True
        )

    def odometry_cb(self, data):
        eulers = euler_from_quaternion([
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ])
        theta = eulers[2]
        self._joint_state.position[0:3] = [
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            theta,
        ]
        self._joint_state.velocity[3:5] = [
            data.twist.twist.linear.x,
            data.twist.twist.angular.z
        ]
        self._joint_state.velocity[0:3] = [
            np.cos(theta) * data.twist.twist.linear.x,
            np.sin(theta) * data.twist.twist.linear.x,
            data.twist.twist.angular.z
        ]

    def joint_states_cb(self, data):
        obs_pos, obs_vel = [], []
        for i, joint in enumerate(data.name):
            if not joint in self._joint_state.name: continue
            idx = self._joint_state.name.index(joint)
            self._joint_state.position[idx] = data.position[i]
            self._joint_state.velocity[idx] = data.velocity[i]

    @property
    def current(self):
        # self._joint_state.header.stamp = rospy.Time.now()
        return self._joint_state #copy.copy(self._joint_state)

if __name__ == "__main__":
    rospy.init_node('joint_states_filter')
    node = JointStatesKF()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass

