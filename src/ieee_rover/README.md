# IEEE Rover

<!-- Builds symbolic links for all projects in ws -->
colcon build --symlink-install
<!-- Applys ROS2 overlay into terminal -->
source install/setup.bash

<!-- Launch Gazebo (gz) simulator -->
ros2 launch ieee_rover sim_launch.py
<!-- Launch a ros_gz_bridge that translates gz lidar messages over /lidar to /lidar_scan topics -->
ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan --ros-args -r /lidar:=/laser_scan
<!-- Listen to /laser_scan and echo messages -->
ros2 topic echo /laser_scan