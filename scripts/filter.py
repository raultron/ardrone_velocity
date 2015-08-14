__author__ = 'racuna'
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

h = numpy.array([0.01616587, 0.03794489, 0.09311559, 0.15589178, 0.19688187, 0.19688187, 0.15589178,  0.09311559,  0.03794489,  0.01616587])
input_buffer = numpy.array([1024.0,2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0])

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %d", data.header.seq)
    pub.publish(data)

def filter(new_value):
    input_buffer[0] = new_value
    size_filter = 10
    result = 0.0
    for i in range(0,size_filter):
        result += input_buffer[i]*h[i]
    for i in range(size_filter-1,0,-1):
        input_buffer[i] = input_buffer[i-1]
        print input_buffer
    print result
