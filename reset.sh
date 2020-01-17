#Disable drone
rostopic pub -1 /enable std_msgs/Bool False 
echo Disabled
#Reset Drone
rosservice call /gazebo/reset_simulation "{}"
echo Reset
#Resetting arms roll, pitch, and yaw PID to zero  

rostopic pub -1 /drone/front_arm_position_controller/command std_msgs/Float64 0 
rostopic pub -1 /drone/back_arm_position_controller/command std_msgs/Float64 0 

rostopic pub -1 /roll/setpoint std_msgs/Float64 0 
rostopic pub -1 /pitch/setpoint std_msgs/Float64 0
rostopic pub -1 /yaw/setpoint std_msgs/Float64 0

echo Zeroed

#Disabling height PID, and setting setpoint to zero
rosparam set /height/controller/pid_enable_topic False
rostopic pub -1 /height/setpoint std_msgs/Float64 0

echo Height PID Disabled
echo Done
