import rospy
import tf
import numpy as np
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates, ModelStates

class wingSim():
    def __init__(self):
        self.wing_update = 0
        self.wing_lift = 0
        self.wing_drag = 0
        self.drone_vel = [0,0,0]
        self.FRW_quat = [0,0,0,0]
        self.FLW_quat = [0,0,0,0]
        self.BRW_quat = [0,0,0,0]
        self.BLW_quat = [0,0,0,0]
        front_pos = rospy.Subscriber("/drone/front_arm_position_controller/command",Float64,self.wing_update)
        model_pos = rospy.Subscriber("/gazebo/model_states",ModelStates,self.pos_update)
        link_pos = rospy.Subscriber("/gazebo/link_states",LinkStates,self.pos_update)

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
                
    def wing_update(self,data):
        self.wing_angle = float(np.degrees(data.data))

        self.wing_lift = (-.00035*(self.wing_angle**2))+(.03097*self.wing_angle)+.04467

        self.wing_drag = (.00000001981*(self.wing_angle**4))+(-.000008304*(self.wing_angle**3))+(.0008272*(self.wing_angle**2))+(-.009907*self.wing_angle)+.05193
        return
    def pos_update(self,data):
        names = data.name
        for i in range(len(names)):
            if names[i] == 'car'
                self.drone_vel = [data.twist[i].linear.x,data.twist[i].linear.y,data.twist[i].linear.z]
            if names[i] == 'car::front_right_wing'
                self.FRW_quat = [data.twist[i].linear.x,data.twist[i].linear.y,data.twist[i].linear.z]
            if names[i] == 'car::front_left_wing'
                self.FLW_quat = [data.twist[i].linear.x,data.twist[i].linear.y,data.twist[i].linear.z]
            if names[i] == 'car::back_right_wing'
                self.BRW_quat = [data.twist[i].linear.x,data.twist[i].linear.y,data.twist[i].linear.z]
            if names[i] == 'car::back_left_wing'
                self.BLW_quat = [data.twist[i].linear.x,data.twist[i].linear.y,data.twist[i].linear.z]
        return

if __name__ == '__main__':
    rospy.init_node("wingsim",anonymous=True)
    try:
        wing = wingSim()
    except rospy.ROSInterruptException:
        pass
