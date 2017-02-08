#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from ardrone_autonomy.msg import Navdata
import rosbag
from  scipy import signal
import scipy
import matplotlib.pyplot as plt
from numpy import cos, sin, pi, absolute, arange

bag = rosbag.Bag('2015-08-14-12-29-31.bag')
vel = list()
time = list()
for topic, msg, t in bag.read_messages('/ardrone/navdata'):
    vel.append(msg.vx)
    time.append(msg.header.stamp)
bag.close()
sample_rate = 25 #1/0.004

vel_out_median = signal.medfilt(vel,21)
#
# plt.plot(vel)
# plt.plot(vel_out)
# plt.show()

# The Nyquist rate of the signal.
nyq_rate = sample_rate / 2.0

# The desired width of the transition from pass to stop,
# relative to the Nyquist rate.  We'll design the filter
# with a 5 Hz transition width.
width = 20.0/nyq_rate

# The desired attenuation in the stop band, in dB.
ripple_db = 20.0

# Compute the order and Kaiser parameter for the FIR filter.
N, beta = signal.kaiserord(ripple_db, width)

# The cutoff frequency of the filter.
cutoff_hz = 1.0

# Use firwin with a Kaiser window to create a lowpass FIR filter.
#taps = signal.firwin(N, cutoff_hz/nyq_rate, window=('kaiser', beta))

#Use a predifined filter order
taps = signal.firwin(30, cutoff_hz/nyq_rate)
print taps

# Use lfilter to filter x with the FIR filter.
vel_out = signal.lfilter(taps, 1.0, vel)
plt.plot(vel_out)
plt.plot(vel)
plt.plot(vel_out_median)

plt.figure(2)
plt.clf()
w, h = signal.freqz(taps, worN=8000)
plt.plot((w/pi)*nyq_rate, absolute(h), linewidth=2)
plt.xlabel('Frequency (Hz)')
plt.ylabel('Gain')
plt.title('Frequency Response')
plt.ylim(-0.05, 1.05)
plt.grid(True)
plt.show()


