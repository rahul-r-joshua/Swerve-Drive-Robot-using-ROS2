ros2 launch rocker_bringup simulated_robot.launch.py use_im_time:=true

ros2 run rocker_controller simple_follow_path.py 

ros2 run rocker_controller simple_pub_circle.py 

ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=key_vel


