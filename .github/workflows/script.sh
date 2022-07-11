#!/bin/bash
sudo apt-get install -y python3-catkin-tools #install catkin_tools
source /opt/ros/noetic/setup.bash
cd karen_ws
sudo rosdep init || echo "rosdep was initialized already, continue regardlessly"
rosdep update && rosdep install --from-paths src --ignore-src -y
catkin build && source devel/setup.bash
roscd fabrics_bridge/test
chmod +x test_fabrics_node && ./test_fabrics_node
