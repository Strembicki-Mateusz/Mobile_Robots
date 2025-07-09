#!/usr/bin/env python

# run: python3 ros2_lidar_subscriber.py 5  (where 5 is robot number)
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

from time import sleep

beam_half_angle=7.5 # half of angular beam width
dataToPlot = []
ax = plt.gca()
ax.set_aspect('equal')
ax.set_xlim([-6,6])
ax.set_ylim([-6,6])
# 
# A function to calculate Cartesian coordinates to polar
#  result: a tuple (rho,phi)
#  rho - radius, phi - angle in degrees
def cart2pol(x, y):
    #TODO: calculations
    rho = math.sqrt(x**2 + y**2)
    phi = math.degrees(math.asin(y/rho))
    return[rho, phi]

# A function to transform polar  coordinates to Cartesian
# input angle in degrees
# returns a tuple (x,y)
def pol2cart(rho, phi):
    #TODO: calculations
    x = rho*math.cos(math.radians(phi))
    y = rho*math.sin(math.radians(phi))
    return[x, y]

# plotting data
def plotwedges(ax,data):
    pol=[cart2pol(x[0],x[1]) for x in data ]
    for item in pol:
        #print item[0],item[1]
        wedge = mpatches.Wedge([0,0], item[0], item[1]-beam_half_angle, item[1]+beam_half_angle, alpha=0.4, ec="black",fc="CornflowerBlue")
        ax.add_patch(wedge)



def plotarrows(ax,arrlist):
    y=[[0,0]+x for x in arrlist ]
    soa =np.array(y) 
    X,Y,U,V = zip(*soa)
    ax.quiver(X,Y,U,V,angles='xy',scale_units='xy',scale=1)




def callback_scan(msg):
    ## extract ranges from message
    scan=list(msg.ranges)
    dataToPlot = []

    beam_half_angle = msg.angle_increment/2
    dataToPlot.clear()

    for i in range(0,len(scan)):
        if scan[i] < 6:
            dataToPlot.append(pol2cart(scan[i], math.degrees(msg.angle_min + msg.angle_increment * i)))

    ax.cla()
    plotarrows(ax,dataToPlot)
    ax.set_xlim([-6,6])
    ax.set_ylim([-6,6])
    plt.draw()
    plt.pause(0.0001)

    plt.ion()
    plt.show()


def main(args=None):
    if len(sys.argv) < 2:
        sys.stderr.write('Usage: sys.argv[0] \n')
        sys.exit(1)
    rclpy.init()
    nr=sys.argv[1]

    node = Node('listener')

    # Subscribe topics and bind with callback functions
    node.create_subscription(LaserScan, f"/pioneer{nr}/scan", callback_scan, 10)

    # spin(node) simply keeps python from exiting until this node is stopped
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
