# Human-Robot-Object-Handover-using-Mixed-Reality

## Installation Requirements
1. ROS2 Galactic: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html#
2. Unity-Robotics-Hub : https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/README.md
3. Interbotix API: https://www.trossenrobotics.com/docs/interbotix_xsarms/ros_interface/ros2_software_setup.html

For the Installation of the HoloLens application see ???

## Package installation
1. Clone repository
1. Create workspace or choose existing workspace
2. Move the two folders `custom_ctrl` and `custom_ctrl_msgs` into the `src` folder of the workspace
3. colcon build packages
4. make sure your workspace is sourced

## Startup
### Checklist

- Connect robot power cable and micro-usb cable
- Make sure the robot is connected to the virtual machine and not the host.
- You can check the connection with the command: `ls /dev | grep ttyDXL`  
The connection works if `ttyDXL`  is printed.
- Connect the HoloLens and your computer to the mobile hotspot
- Run the terminal commands
- Start the app on the HoloLens

### Necessary terminal commands

- `ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250s use_sim:=true`
- `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=192.168.240.129` (change ROS_IP to IP of your system)
- `ros2 launch custom_ctrl robot_framework_launch.py`
- `ros2 run custom_ctrl task_controller`
