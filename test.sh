source ~/drone_simulation/test_ws/devel/setup.bash

rostopic pub /roll/setpoint std_msgs/Float64 0 &
#rostopic pub /yaw/setpoint std_msgs/Float64 0 &
rostopic pub /pitch/setpoint std_msgs/Float64 0 &

rostopic pub /drone/front_arm_position_controller/command std_msgs/Float64 "data: 0" &
rostopic pub /drone/back_arm_position_controller/command std_msgs/Float64 "data: 0" &
sleep 1
rostopic pub -1 /enable std_msgs/Bool "data: true"
rostopic pub -1 /height/setpoint std_msgs/Float64 "data: 2.0"
sleep 4
rostopic pub /drone/front_arm_position_controller/command std_msgs/Float64 -- .1 &
#rostopic pub /drone/back_arm_position_controller/command -- std_msgs/Float64 6.18 &
sleep 1
rostopic pub /roll/setpoint std_msgs/Float64 .3 &
