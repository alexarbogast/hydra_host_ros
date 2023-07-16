# hydra_host_ros
Host-side ros packages for running the Hydra multi-robot system 

These packages provide MoveIt integration for the [Tormach Za6](https://tormach.com/machines/robots.html) and the hydra system. They run on a host machine running the ros master.
They communicate with the docker containers running on the pathpilot robot controller.

## Installation
```sh
mkdir hydra_host_ws/src && cd hydra_host_ws/src
git clone --recurse-submodules git@github.com:alexarbogast/hydra_host_ros.git
cd ../ && catkin build
```
## Running the Za6
After setting up the [ROS network](http://wiki.ros.org/ROS/NetworkSetup), start the ros master on the host machine.
```sh
roscore
```
Next, start the hal hardware interface and ros controllers on the robot computer in the tormach_ros_dev container found at [za_docker](https://github.com/alexarbogast/za_docker)

To engage the motors on the robot, you will need to publish a state_cmd to the hardware interface.
```sh
rostopic pub /za/hal_io/state_cmd std_msgs/UInt32 "data: 2"
```
Start the motion planning scene with:
```sh
roslaunch za_moveit_planning moveit_planning.launch
```
This will bring up rviz where you can leverage the MoveIt motion planning plugin.

## Running the Hydra System
Start the ros master on the host machine.
```sh
roscore
```
Bringup up the robot hardware packages under the namespaces `rob1` and `rob2`.
Engage the motors on both robots.
```sh
rostopic pub /rob1/hal_io/state_cmd std_msgs/UInt32 "data: 2"
rostopic pub /rob2/hal_io/state_cmd std_msgs/UInt32 "data: 2"
```
Launch the MoveIt planning scene
```sh
roslaunch hydra_moveit_planning moveit_planning.launch
```
You can run the coordinated motion demo with the following command
```sh
 rosrun hydra_moveit_planning hydra_demo
```
