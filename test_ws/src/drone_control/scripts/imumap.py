#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

class Remap():
    def __init__(self):
        self.yaw = 0
        yaw = rospy.Publisher("/yaw/state",Float64,queue_size=10)
        rate = rospy.Rate(100)
        sub = rospy.Subscriber("/drone/imu_data",Imu,self.callback)
        while not rospy.is_shutdown():
            yaw.publish(self.yaw)
    
    def callback(self, data):
        self.yaw = data.angular_velocity.z
        return

if __name__ == '__main__':
    rospy.init_node("imu_remap",anonymous=True)
    try:
        remap = Remap()
    except rospy.ROSInterruptException:
        pass

