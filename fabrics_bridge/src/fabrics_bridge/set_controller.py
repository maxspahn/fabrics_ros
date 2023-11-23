# ros imports
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class StiffnessNode:
    def __init__(
            self,
        ):
        self._dq = np.zeros(7)
        self._dq_des = np.zeros(7)
        rospy.init_node("set_stiffness")
        self.establish_ros_connections()
        self._rate = rospy.Rate(500)
        rospy.sleep(1.0)
        rospy.loginfo("Starting RecordSkillNode as record_skill.")

        self.set_controller()
        

    def establish_ros_connections(self):
        self._ki_publisher = rospy.Publisher(
            "/compliant_joint_velocity_controller/controller_ki",
            Float64MultiArray,
            queue_size=1
        )
        self._diff_publisher = rospy.Publisher("/diff", Float64MultiArray, queue_size=1)
        self._kp_publisher = rospy.Publisher(
            "/compliant_joint_velocity_controller/controller_kp",
            Float64MultiArray,
            queue_size=1
        )
        rospy.Subscriber('/joint_states_filtered', JointState, self.joint_states_callback)
        rospy.Subscriber('/panda_joint_velocity_controller/command', Float64MultiArray, self.command_callback)

    def joint_states_callback(self, msg: JointState):
        self._dq[0:7] = msg.velocity[5:12]

    def command_callback(self, msg: Float64MultiArray):
        self._dq_des[0:7] = msg.data[0:7]


    def set_controller(self):
        kp = [10, 20, 30, 40, 10, 10, 5]
        ki = [2, 2, 2, 2, 0.5, 1, 0.5]
        #kp = [0.1] * 7
        #ki = [0] * 7
        ki_msg = Float64MultiArray(data=ki)
        kp_msg = Float64MultiArray(data=kp)
        rospy.loginfo("Setting controller gains.")
        for i in range(10):
            self._ki_publisher.publish(ki_msg)
            self._kp_publisher.publish(kp_msg)

    def publish_difference(self):
        diff_msg = Float64MultiArray()
        diff_msg.data = self._dq - self._dq_des
        self._diff_publisher.publish(diff_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.set_controller()
            self.publish_difference()
            self._rate.sleep()

if __name__ == "__main__":
    controller_setter = StiffnessNode()
    try:
        controller_setter.run()
    except rospy.ROSInterruptException:
        pass
