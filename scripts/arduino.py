#!/usr/bin/env python
import math
import rospy
import serial
import sys
from std_msgs.msg import String
from geometry_msgs.msg import TwistWithCovarianceStamped, Twist
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

cnt2mps=24  # encoder counts per sample to meters per sec conversion
r=0.15      # radius from body center to tread center

def sub_callback(msg):
    global g_ser
    rospy.loginfo(rospy.get_caller_id()+'I heard "%f"', msg.linear.x)
    vr = (msg.linear.x - msg.angular.z*r)*cnt2mps
    vl = (msg.linear.x + msg.angular.z*r)*cnt2mps
    g_ser.write("{:d},{:d}\n".format(int(vr), int(vl)).encode())

def main(args=None):
    global g_ser
    rospy.init_node('arduino')

    publisher = rospy.Publisher('vel',TwistWithCovarianceStamped, queue_size=10)
    imu_publisher = rospy.Publisher('imu',Imu, queue_size=10)
    rospy.Subscriber("cmd_vel", Twist, sub_callback)

    msg = TwistWithCovarianceStamped()
    msg.header.frame_id = "base_link" 
    msg.twist.covariance=[0.1,0,0,0,0,0,\
                          0,0.1,0,0,0,0,\
                          0,0,0.1,0,0,0,\
                          0,0,0,0.1,0,0,\
                          0,0,0,0,0.1,0,
                          0,0,0,0,0,0.1]

    imu_msg = Imu()
    imu_msg.header.frame_id = "base_link" 
    imu_msg.orientation_covariance=[\
                          0.1,0,0,\
                          0,0.1,0,\
                          0,0,0.1]
    imu_msg.linear_acceleration_covariance=[\
                          0.1,0,0,\
                          0,0.1,0,\
                          0,0,0.1]


    i = 0
    g_ser=serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        line=g_ser.readline(100)
        vals=line.decode().split(',')
        if(len(vals) < 2):
            print("Got ",len(vals)," vals from ",line)
            continue
        #print("Got ",vals)
        try:
            vr=float(vals[0])
            vl=float(vals[1])
            msg.twist.twist.linear.x= float((vr+vl)/2/cnt2mps)
            msg.twist.twist.angular.z= float((vl-vr)/r/cnt2mps)
            rospy.loginfo('Serial Rx: %s'%str(line))
            msg.header.stamp = rospy.Time.now()
            publisher.publish(msg)
        except Exception:
            print("invalid encoder values", sys.exc_info()[0])
        try:
            mx=float(vals[2])
            my=float(vals[3])
            yaw=math.atan2(my,mx)
        #    print("yaw=",yaw)
            q=quaternion_from_euler(0,0,yaw)
            imu_msg.orientation.x = q[0]
            imu_msg.orientation.y = q[1]
            imu_msg.orientation.z = q[2]
            imu_msg.orientation.w = q[3]
            imu_msg.linear_acceleration.x=float(vals[5])
            imu_msg.linear_acceleration.y=float(vals[6])
            imu_msg.linear_acceleration.z=float(vals[7])
            imu_msg.header.stamp = msg.header.stamp
            imu_publisher.publish(imu_msg)
        except Exception as e: 
            print("invalid compass values", sys.exc_info()[0])
            print(e)

        rate.sleep()

if __name__ == '__main__':
    main()
