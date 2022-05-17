# fabrics_ros

This is a ROS wrapper for implementing [fabrics](https://github.com/maxspahn/fabrics)

The list of packages:
- fabrics_bridge: provides a one-way bridge to enable ROS to use fabrics planner for robot motion planning
- fabrics_msgs: contains custom messages used in fabrics_ros
- fabrics_processing: processes output from fabrics planner, and publishes velocity commands to a robot

## Requirements

- Python <3.10
- ROS Melodic

Note: These requirements are tested by the authors at the time of writing, other versions have not been tested yet.

## Installation

### Install ROS dependencies

```bash
mkdir -p ~/fabrics_ros_ws/src && cd ~/fabrics_ros_ws/src
git clone git@github.com:maxspahn/fabrics_ros.git
sudo rosdep init
rosdep update
cd ..
rosdep install --from-paths src --ignore-src -y
```



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
> Pass `use_it_marker:=true`, if you want to use an interactive marker as a goal

```bash
roslaunch fabrics_bridge fabrics_demo.launch
```


If you have a perception module which can publish data to `/planning_goal` and `/planning_obs`, run:

```bash
roslaunch fabrics_bridge fabrics_node.launch
```


Then, to process output from fabrics, and publish velocity commands to `/panda_joint_velocity_controller/command`, run:

```bash
roslaunch fabrics_processing fabrics_processing.launch
```

## Real Robot


Same steps as in the simulation section except the part launching `albert_gazebo_navigation.launch` 

