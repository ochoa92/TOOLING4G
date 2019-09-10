#!/usr/bin/python

import rospy
import numpy as np
import array as arr
from matplotlib import pyplot
from scipy import signal

def main():

    rospy.init_node("udrilling_operator_data")

    # Open the file back and read the contents
    path = "/home/helio/catkin_ws/src/franka_ros/franka_udrilling/scripts/refs_operator/05"
    f = open(path, 'r')
    t = []
    z = []
    delimiter = " "
    for i, line in enumerate(f):
        if i > 2:
            t.append(float(line.split(delimiter)[1]))
            z.append(float(line.split(delimiter)[6]))
    f.close()

    # Create a lowpass butterworth filter.
    order = 8
    cutoff = 0.01
    b, a = signal.butter(order, cutoff, btype='lowpass')
    # Use filtfilt to apply the filter.
    y = signal.filtfilt(b, a, z)

    # plot
    pyplot.figure(1, figsize=(10, 10))
    pyplot.plot(t, z, 'b', linewidth = 1.75)
    pyplot.plot(t, y, 'r--', linewidth = 1.75)
    pyplot.grid(True)
    pyplot.xlabel('t')
    pyplot.ylabel('z')
    pyplot.show()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
