rostopic pub -1 /drone/front_arm_position_controller/command std_msgs/Float64 0
rostopic pub -1 /drone/back_arm_position_controller/command std_msgs/Float64 0

rostopic pub -1 /roll/setpoint std_msgs/Float64 0
rostopic pub -1 /pitch/setpoint std_msgs/Float64 0

rostopic pub -1 /height/setpoint std_msgs/Float64 2 
#rostopic pub -1 /enable std_msgs/Bool True

sleep 10

#rostopic pub /roll/setpoint std_msgs/Float64 .1 &
#rostopic pub /drone/front_arm_position_controller/command std_msgs/Float64 .1 &
#rostopic pub /drone/back_arm_position_controller/command std_msgs/Float64 6.18 &
