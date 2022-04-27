# ros_robot
Ros Files to get robot to move...kinda

## Dependencies
In addtion to this package, these need to be installed as well:
- [hector_slam](https://github.com/tu-darmstadt-ros-pkg/hector_slam/tree/melodic-devel) 
- [RPLidar](https://github.com/Slamtec/rplidar_ros)

## Terminal Programs
To upload from the command line, arduino-cli needs to be installed.
This can be done one of two ways:
1. Install through Brew
   - Install [brew](https://docs.brew.sh/Homebrew-on-Linux)
   - Install [arduino-cli](https://arduino.github.io/arduino-cli/0.21/installation/) through brew
2. Install through curl
   - Install [arduino-cli](https://arduino.github.io/arduino-cli/0.21/installation/) through curl

## Editor
There are many editor choices, below are two options:
1. VIM/NVIM through the terminal
2. [VSCode through SSH](https://code.visualstudio.com/docs/remote/ssh)


### How to Use
1. Once above steps are done, clone this repository
2. Drill to the robotedit2 folder
   1. Compile file ```arduino-cli -b arduino:avr:uno robotedit2.ino```
   2. Upload the compiled file ```arduino-cli -p /dev/ttyACM0 robotedit2.ino``` You may need to change the port
3. CD to the catkin workspace and catkin build everything
4. ```Source devel/setup.bash```
5. ```roslaunch ros_robot robot_bare.launch```
6. Wait for the transforms, maps, etc to load and for the constant serial stream
7. ```roslaunch ros_robot move_base.launch```
8. Open RVIZ and give the robot an initial 2D estimate
9. Set a navigation goal and watch it go



