# odompub.py
Odometry transforms were being published but a few nodes require an odom topic. 
Because of the accuracy, scanmatch_odom from hector_slam was used as the primary odom topic. odompub.py is used to publish this to the right topic.