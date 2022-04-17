#!/usr/bin/env python
import rospy
import sys
from sensor_msgs.msg import NavSatFix

global publisher
def sub_callback(msg):
    global publisher
    msg.header.frame_id="gps_link"
    publisher.publish(msg)

def main(args=None):
    global publisher
    rospy.init_node('gps_tf_fix')

    publisher = rospy.Publisher('gps/fix',NavSatFix, queue_size=10)
    rospy.Subscriber("gps/raw", NavSatFix, sub_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
