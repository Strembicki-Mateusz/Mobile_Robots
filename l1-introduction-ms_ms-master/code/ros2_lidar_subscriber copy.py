#!/usr/bin/env python

# run: python3 ros2_lidar_subscriber.py 5  (where 5 is robot number)
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

def callback_scan(msg):
    ## extract ranges from message
    scan=list(msg.ranges)
    # print("  Scan min: %f  front: %f" % ( min(scan),scan[362]))
    print(f"Min angle: {math.degrees(msg.angle_min)}, max angle: {math.degrees(msg.angle_max)}, no. of samples {len(msg.ranges)}")
    print ()


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
