#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import rosbag
from  scipy import signal
import scipy
import matplotlib.pyplot as plt
from numpy import cos, sin, pi, absolute, arange
import numpy

#Filter coefficients
h = numpy.array([ 0.00498902,  0.00567655,  0.00768429,  0.01092849,  0.01526534,  0.02049766,
  0.02638413,  0.03265082,  0.0390043,   0.04514569,  0.05078516,  0.05565588,
  0.05952694,  0.06221459,  0.06359114,  0.06359114,  0.06221459,  0.05952694,
  0.05565588,  0.05078516,  0.04514569,  0.0390043,   0.03265082,  0.02638413,
  0.02049766,  0.01526534,  0.01092849,  0.00768429,  0.00567655,  0.00498902])

#Input buffer for the filter
input_buffer = numpy.array([0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %d", data.header.seq)
    output = filter(data.twist.twist.linear.x)
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    #msg.header.stamp = data.header.stamp
    msg.twist.twist.linear.x = output
    pub.publish(msg)

def filter(new_value):
    input_buffer[0] = new_value
    size_filter = 30
    result = 0.0
    for i in range(0,size_filter):
        result += input_buffer[i]*h[i]
    for i in range(size_filter-1,0,-1):
        input_buffer[i] = input_buffer[i-1]
    return result

if __name__ == '__main__':
    pub = rospy.Publisher('/pid/filtered_velocity', Odometry, queue_size=1, tcp_nodelay=True)
    rospy.init_node('filter_calibration', anonymous=True)
    rospy.Subscriber("/ardrone/odometry", Odometry, callback, tcp_nodelay=True)

    rospy.spin()
    