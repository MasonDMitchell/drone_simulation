#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

class Remap():
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        roll = rospy.Publisher("/roll/state",Float64,queue_size=10)
        pitch = rospy.Publisher("/pitch/state",Float64,queue_size=10)
        yaw = rospy.Publisher("/yaw/state",Float64,queue_size=10)
        rate = rospy.Rate(100)
        sub = rospy.Subscriber("/drone/imu_data",Imu,self.callback)
        while not rospy.is_shutdown():
            roll.publish(self.roll)
            pitch.publish(self.pitch)
            yaw.publish(self.yaw)
    
    def callback(self, data):
        self.roll = data.angular_velocity.x
        self.pitch = data.angular_velocity.y
        self.yaw = data.angular_velocity.z
        return

if __name__ == '__main__':
    rospy.init_node("imu_remap",anonymous=True)
    try:
        remap = Remap()
    except rospy.ROSInterruptException:
        pass

