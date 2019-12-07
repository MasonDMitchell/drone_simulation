#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

class Remap():
    def __init__(self):
        self.throttle = 0
        self.roll = 0
        self.pitch = 0
        self.yaw_placeholder = 0
        self.yaw = 0
        self.arm = 0
        throttle = rospy.Publisher("/throttle",Float64,queue_size=10)
        roll = rospy.Publisher("/roll/setpoint",Float64,queue_size=10)
        pitch = rospy.Publisher("/pitch/setpoint",Float64,queue_size=10)
        yaw = rospy.Publisher("/yaw/setpoint",Float64,queue_size=10)
        front_arm = rospy.Publisher("/drone/back_arm_position_controller/command",Float64,queue_size=10)
        back_arm = rospy.Publisher("/drone/front_arm_position_controller/command",Float64,queue_size=10)
        rate = rospy.Rate(100)
        sub = rospy.Subscriber("/joy", Joy, self.callback)
        while not rospy.is_shutdown():
            throttle.publish(self.throttle)
            roll.publish(-self.roll)
            pitch.publish(-self.pitch)
            yaw.publish(self.yaw)
            front_arm.publish(self.arm)
            back_arm.publish(-self.arm)
        return
    def callback(self, data):
        self.throttle = (-data.axes[0]+1)/2
        self.roll = data.axes[1]/2
        self.pitch = data.axes[2]/2
        #self.yaw_placeholder = self.yaw_placeholder + (.01*data.axes[3]*3.14159)
        #self.yaw = (self.yaw_placeholder+3.14159)%  (2*3.14159) - 3.14159
        self.yaw = data.axes[3]*2
        self.arm = data.axes[4]*(3.14159/4)
        return 

if __name__ == '__main__':
    rospy.init_node("joy_remap",anonymous=True)
    try:
        remap = Remap()
    except rospy.ROSInterruptException:
        pass
