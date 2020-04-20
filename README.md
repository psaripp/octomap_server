# Octomap Server 

Modified version of [`map_server`](https://github.com/ros-planning/navigation/tree/kinetic-devel/map_server) example from ROS Moveit. 

## Prerequisites

- [octomap](https://github.com/OctoMap/octomap.git), and [octomap_msgs](https://github.com/OctoMap/octomap_msgs.git)
- Optional: [ROS Websocket bridge](http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge)

## Usage

After `catkin_make` and initialize workspace:

```
rosrun octomap_server map_server [-l] [-b | -f] map.(bt|ot)
    -f forces to send a full map
    -b forces to send a binary map
    -l latch published message
```

Test maps can be found in `test` folder.