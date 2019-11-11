#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
import tf


class Remap():
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.quat = []
        self.height = 0
        self.euler = []
        link_pos = rospy.Subscriber("/gazebo/model_states",LinkStates,self.pos_update)

        rate = rospy.Rate(100)
        height = rospy.Publisher("/height/state",Float64,queue_size=10)
        roll = rospy.Publisher("/roll/state",Float64,queue_size=10)
        pitch = rospy.Publisher("/pitch/state",Float64,queue_size=10)
        yaw = rospy.Publisher("/yaw/state",Float64,queue_size=10)
        while not rospy.is_shutdown():
            height.publish(self.height)
            roll.publish(self.roll)
            pitch.publish(self.pitch)
            yaw.publish(self.yaw)
        return
    def pos_update(self,data):
        names = data.name
        for i in range(len(names)):
            if names[i] == 'car':
                self.quat = [data.pose[i].orientation.x,data.pose[i].orientation.y,data.pose[i].orientation.z,data.pose[i].orientation.w]
                self.height = data.pose[i].position.z
                self.qv_eul(self.quat)
        return

    def qv_eul(self,q1):
        self.euler = tf.transformations.euler_from_quaternion(q1)
        self.roll = self.euler[0] 
        self.pitch = self.euler[1]
        self.yaw = self.euler[2]
        return tf.transformations.euler_from_quaternion(q1)

if __name__ == '__main__':
    rospy.init_node("quat_to_euler",anonymous=True)
    try:
        remap = Remap()
    except rospy.ROSInterruptException:
        pass
