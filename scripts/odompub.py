#!/usr/bin/env python
import rospy
import sys
from nav_msgs.msg import Odometry


global publisher
def sub_callback(msg):
    global publisher
    msg.header.frame_id="odom"
    publisher.publish(msg)

def main(args=None):
    global publisher
    rospy.init_node('odom_fixer')

    publisher = rospy.Publisher('odom',Odometry, queue_size=10)
    rospy.Subscriber("scanmatch_odom", Odometry, sub_callback)

    rospy.spin()

if __name__ == '__main__':
    main()