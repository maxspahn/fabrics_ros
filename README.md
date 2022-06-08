# fabrics_ros

This is a ROS wrapper for implementing [fabrics](https://github.com/maxspahn/fabrics)

> ⚠️ `fabrics` is still a private repository at the moment 


The list of packages:
- fabrics_bridge: provides a one-way bridge to enable ROS to use fabrics planner for robot motion planning
- fabrics_msgs: contains custom messages used in fabrics_ros
- fabrics_processing: processes output from fabrics planner, and publishes velocity commands to a robot

## Requirements

- Python 3.6, <3.10
- ROS Melodic

Note: These requirements are tested by the authors at the time of writing, other versions have not been tested yet.

## Installation

### Install ROS dependencies

```bash
mkdir -p ~/fabrics_ros_ws/src && cd ~/fabrics_ros_ws/src
git clone git@github.com:maxspahn/fabrics_ros.git
git clone -b 0.6.5 https://github.com/ros/geometry2 #tf for python3
sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy #prerequisites to use python3 with ROS
sudo rosdep init
rosdep update
cd ..
rosdep install --from-paths src --ignore-src -y
catkin build --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
```

> Instructions above for building tf in python3 is taken from this [post](https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/). Alternatively, you can create a new tf workspace and use workspace overlaying.

> **_NOTE:_**  If you encounter this error when running `sudo rosdep init`:  
*`ERROR: default sources list file already exists`*, please ignore it and continue with the rest of the steps.

### Install fabrics dependencies


Since there are some python packages not mapped to ROS dependencies, i.g. `python3-pykalman`, non-ROS Python packages will be installed using `pip3`. See [rosdep/python.yaml](https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml) for the entire list.


```bash
cd ~/fabric_ros_ws/src/fabrics_ros
pip3 install -r requirements.txt
```





## Simulation

To start an Albert simulation environment using velocity control mode:

```bash
roslaunch albert_gazebo albert_gazebo_navigation.launch panda_control_mode:=velocity
```

If you don't have a real goal and obstacles published/detected, run:

```bash
roslaunch fabrics_bridge fabrics_interactive_marker.launch
```


If you have a module which can publish data to `/planning_goal` and `/planning_obs`, run:

```bash
roslaunch fabrics_bridge fabrics_node.launch
```
An example client node that can publish `/planning_goal` and `/planning_obs`:


```bash
rosrun fabrics_bridge client_node
```



Then, to process output from fabrics, and publish velocity commands to `/panda_joint_velocity_controller/command`, run:

```bash
roslaunch fabrics_processing fabrics_processing.launch
```

## Real Robot


Same steps as in the simulation section except the part launching `albert_gazebo_navigation.launch` 

