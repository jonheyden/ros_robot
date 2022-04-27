# edited_unused
Files that were generated during the project but ultimately left by the wayside
# unused_launch
Launch files that were pulled but ultimately never used

# robot_bare.launch
Used to launch the robot including:
1. tf.launch
2. arduino.launch
3. localization/localization_copy.launch
4. gps.launch
5. rplidar.launch (from slamtec/rplidar_ros)
6. hector.launch
7. odompub.py node

# move_basic.launch
Rudimentary path planner/goal taker that will rotate the bot and move to a point. Works fairly well

# move_base.launch
Allegedly better than move_basic, will generate cost maps and use the global/local planners to attempt to efficiently move to a point. Needs to be endlessly configured and tuned in the config folder that includes a handful of yaml files.

# keyboard.launch
Used to launch the keyboard teleop to manually control the bot. Can be used with robot_bare to generate a map from hector_slam, which can then be saved.

# jetson_csi_cam.launch
launches the front facing camera