#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Wrench 
from gazebo_msgs.msg import LinkStates

class Control():
    def __init__(self):
        self.throttle = 0
        self.roll = 0
        self.pitch = 0 
        self.yaw = 0 
        self.constant = 8.5
        self.height = 0
        self.enable = False
        self.FR_quat = [0,0,0,0]
        self.FL_quat = [0,0,0,0]
        self.BR_quat = [0,0,0,0]
        self.BL_quat = [0,0,0,0]
        FR_pid_val = 0
        FL_pid_val = 0
        BR_pid_val = 0
        BL_pid_val = 0 

        link_pos = rospy.Subscriber("/gazebo/link_states",LinkStates,self.pos_update)
        #drone_pos = rospy.Subscriber("/gazebo/model_states",LinkStates,self.pos_update)

        throttle = rospy.Subscriber("/throttle",Float64,self.throttle_update)
        roll_control = rospy.Subscriber("/roll/control_effort",Float64,self.roll_update)
        pitch_control = rospy.Subscriber("/pitch/control_effort",Float64,self.pitch_update)
        yaw_control = rospy.Subscriber("/yaw/control_effort",Float64,self.yaw_update)
        height_control = rospy.Subscriber("/height/control_effort",Float64,self.height_update)
        enable = rospy.Subscriber('/enable', Bool,self.enable_update)
        FR_thrust = rospy.Publisher("/drone/front_right_motor_thrust",Wrench,queue_size=10)
        FL_thrust = rospy.Publisher("/drone/front_left_motor_thrust",Wrench,queue_size=10)
        BR_thrust = rospy.Publisher("/drone/back_right_motor_thrust",Wrench,queue_size=10)
        BL_thrust = rospy.Publisher("/drone/back_left_motor_thrust",Wrench,queue_size=10)
         
        FR_thrust_val = Wrench()
        FL_thrust_val = Wrench() 
        BR_thrust_val = Wrench()
        BL_thrust_val = Wrench()
        
        FR_motor = rospy.Publisher('/drone/front_right_motor_velocity_controller/command',Float64,queue_size=10)
        FL_motor = rospy.Publisher('/drone/front_left_motor_velocity_controller/command',Float64,queue_size=10)
        BR_motor = rospy.Publisher('/drone/back_right_motor_velocity_controller/command',Float64,queue_size=10)
        BL_motor = rospy.Publisher('/drone/back_left_motor_velocity_controller/command',Float64,queue_size=10)

        pitch_constant = 1
        roll_constant = 1
        yaw_constant = 1
        torque_constant = .2
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():

            if self.throttle == 0 and self.enable == False:

                FR_thrust_val.force.x = 0 
                FR_thrust_val.force.y = 0 
                FR_thrust_val.force.z = 0
                FR_thrust_val.torque.x = 0
                FR_thrust_val.torque.y = 0
                FR_thrust_val.torque.z = 0

                FL_thrust_val.force.x = 0 
                FL_thrust_val.force.y = 0 
                FL_thrust_val.force.z = 0 
                FL_thrust_val.torque.x = 0
                FL_thrust_val.torque.y = 0
                FL_thrust_val.torque.z = 0

                BR_thrust_val.force.x = 0 
                BR_thrust_val.force.y = 0 
                BR_thrust_val.force.z = 0 
                BR_thrust_val.torque.x = 0
                BR_thrust_val.torque.y = 0
                BR_thrust_val.torque.z = 0

                BL_thrust_val.force.x = 0 
                BL_thrust_val.force.y = 0 
                BL_thrust_val.force.z = 0 
                BL_thrust_val.torque.x = 0
                BL_thrust_val.torque.y = 0
                BL_thrust_val.torque.z = 0

                FR_pid_val = 0
                FL_pid_val = 0
                BR_pid_val = 0
                BL_pid_val = 0 

            else:

                if self.throttle > .8: 
                    self.throttle=.8

                FR_pid_val = self.constant*(self.throttle + self.height - pitch_constant*self.pitch - roll_constant*self.roll + yaw_constant*self.yaw) #-yaw
                FL_pid_val = self.constant*(self.throttle + self.height - pitch_constant*self.pitch + roll_constant*self.roll - yaw_constant*self.yaw) #+yaw
                BR_pid_val = self.constant*(self.throttle + self.height + pitch_constant*self.pitch - roll_constant*self.roll - yaw_constant*self.yaw) #+yaw
                BL_pid_val = self.constant*(self.throttle + self.height + pitch_constant*self.pitch + roll_constant*self.roll + yaw_constant*self.yaw) #-yaw

                if FR_pid_val < .1*self.constant: 
                    FR_pid_val = .1*self.constant
                if FL_pid_val < .1*self.constant: 
                    FL_pid_val = .1*self.constant
                if BR_pid_val < .1*self.constant: 
                    BR_pid_val = .1*self.constant
                if BL_pid_val < .1*self.constant: 
                    BL_pid_val = .1*self.constant

                if FR_pid_val > 1*self.constant: 
                    FR_pid_val = 1*self.constant
                if FL_pid_val > 1*self.constant: 
                    FL_pid_val = 1*self.constant
                if BR_pid_val > 1*self.constant: 
                    BR_pid_val = 1*self.constant
                if BL_pid_val > 1*self.constant: 
                    BL_pid_val = 1*self.constant

                FR_transform = self.qv_mult(self.FR_quat,[0,0,1])
                FL_transform = self.qv_mult(self.FL_quat,[0,0,1])
                BR_transform = self.qv_mult(self.BR_quat,[0,0,1])
                BL_transform = self.qv_mult(self.BL_quat,[0,0,1])
                
                FR_thrust_val.force.x = FR_transform[0]*FR_pid_val
                FR_thrust_val.force.y = FR_transform[1]*FR_pid_val
                FR_thrust_val.force.z = FR_transform[2]*FR_pid_val
                FR_thrust_val.torque.x = (FR_transform[0]*FR_pid_val*torque_constant)
                FR_thrust_val.torque.y = (FR_transform[1]*FR_pid_val*torque_constant)
                FR_thrust_val.torque.z = (FR_transform[2]*FR_pid_val*torque_constant)

                FL_thrust_val.force.x = FL_transform[0]*FL_pid_val
                FL_thrust_val.force.y = FL_transform[1]*FL_pid_val
                FL_thrust_val.force.z = FL_transform[2]*FL_pid_val
                FL_thrust_val.torque.x = -(FL_transform[0]*FL_pid_val*torque_constant)
                FL_thrust_val.torque.y = -(FL_transform[1]*FL_pid_val*torque_constant)
                FL_thrust_val.torque.z = -(FL_transform[2]*FL_pid_val*torque_constant)

                BR_thrust_val.force.x = BR_transform[0]*BR_pid_val
                BR_thrust_val.force.y = BR_transform[1]*BR_pid_val
                BR_thrust_val.force.z = BR_transform[2]*BR_pid_val
                BR_thrust_val.torque.x = -(BR_transform[0]*BR_pid_val*torque_constant)
                BR_thrust_val.torque.y = -(BR_transform[1]*BR_pid_val*torque_constant)
                BR_thrust_val.torque.z = -(BR_transform[2]*BR_pid_val*torque_constant)

                BL_thrust_val.force.x = BL_transform[0]*BL_pid_val
                BL_thrust_val.force.y = BL_transform[1]*BL_pid_val
                BL_thrust_val.force.z = BL_transform[2]*BL_pid_val
                BL_thrust_val.torque.x = (BL_transform[0]*BL_pid_val*torque_constant)
                BL_thrust_val.torque.y = (BL_transform[1]*BL_pid_val*torque_constant)
                BL_thrust_val.torque.z = (BL_transform[2]*BL_pid_val*torque_constant)

            FR_thrust.publish(FR_thrust_val)
            FL_thrust.publish(FL_thrust_val)
            BR_thrust.publish(BR_thrust_val)
            BL_thrust.publish(BL_thrust_val)

            #FR_motor.publish(0)
            #FL_motor.publish(0)
            #BR_motor.publish(0)
            #BL_motor.publish(0)
            
        return
    
    def pos_update(self,data):
        names = data.name
        for i in range(len(names)):
            if names[i] == 'car':
                self.drone_quat = [data.pose[i].orientation.x,data.pose[i].orientation.y,data.pose[i].orientation.z,data.pose[i].orientation.w]
            if names[i] == 'car::front_right_prop':
                self.FR_quat = [data.pose[i].orientation.x,data.pose[i].orientation.y,data.pose[i].orientation.z,data.pose[i].orientation.w]
            if names[i] == 'car::front_left_prop':
                self.FL_quat = [data.pose[i].orientation.x,data.pose[i].orientation.y,data.pose[i].orientation.z,data.pose[i].orientation.w]
            if names[i] == 'car::back_right_prop':
                self.BR_quat = [data.pose[i].orientation.x,data.pose[i].orientation.y,data.pose[i].orientation.z,data.pose[i].orientation.w]
            if names[i] == 'car::back_left_prop':
                self.BL_quat = [data.pose[i].orientation.x,data.pose[i].orientation.y,data.pose[i].orientation.z,data.pose[i].orientation.w]

        return

    def qv_mult(self, q1,v1):
        v1 = tf.transformations.unit_vector(v1)
        q2 = list(v1)
        q2.append(0.0)
        return tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q1,q2),tf.transformations.quaternion_conjugate(q1))[:3]

    def qv_eul(self,q1):
        return tf.transformations.euler_from_quaternion(q1)

    def enable_update(self,data):
        self.enable = data.data
        return
    def throttle_update(self, data):
        self.throttle = data.data
        return
    def height_update(self,data):
        self.height = data.data
        return
    def roll_update(self,data):
        self.roll = data.data
        return
    def pitch_update(self,data):
        self.pitch = data.data
        return
    def yaw_update(self,data):
        self.yaw = data.data
        return
if __name__ == '__main__':
    rospy.init_node("pidcontrol",anonymous=True)
    try:
        control = Control()
    except rospy.ROSInterruptException:
        pass
