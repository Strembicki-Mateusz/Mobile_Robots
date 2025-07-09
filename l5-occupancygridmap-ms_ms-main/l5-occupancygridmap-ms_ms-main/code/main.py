#!/usr/bin/env python
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import numpy as np
from nav_msgs.msg import Odometry
import math
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import breshnamm 
import csv
import time

ROOM_WIDTH = 20
ROOM_LENGHT = 20
MAP_WIDTH = 200
MAP_LENGTH = 200
CELL_SIZE = float(MAP_LENGTH)/float(ROOM_LENGHT)

FREE_FIELD_PROB = 0.1
OCCUPIED_FIELD_PROB = 0.9
DEAFULT_FIELD_PROB = 0.5

def cart2pol(x, y):
    rho = math.sqrt(x**2 + y**2)
    phi = math.degrees(math.asin(y/rho))
    return [rho, phi]

def pol2cart(rho, phi):
    x = rho*math.cos(math.radians(phi))
    y = rho*math.sin(math.radians(phi))
    return[x, y]

def yaw_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return yaw

class OccupancyGridMap(Node):
    def __init__(self):
        super().__init__("map")
        self.pioneerNo = 1

        self.scalingFactor = CELL_SIZE

        self.grid = np.full(shape=(MAP_WIDTH, MAP_LENGTH), fill_value=DEAFULT_FIELD_PROB)
        self.scan = np.full(shape=(MAP_WIDTH, MAP_LENGTH), fill_value=DEAFULT_FIELD_PROB)

        self.robotStartPosition = [int(MAP_WIDTH/2), int(MAP_LENGTH/2)]
        self.robotPosition = [int(MAP_WIDTH/2), int(MAP_LENGTH/2)]
        self.robotOrientation = 0.0
        self.grid[self.robotPosition[0]][self.robotPosition[1]] = 1 # tu stoi robot na starcie

        self.fig, self.ax = plt.subplots()
        self.img = self.ax.imshow(self.grid, interpolation="nearest", cmap='Blues', norm=mcolors.Normalize(vmin=0, vmax=1))
        self.colorbar = self.fig.colorbar(self.img)

        self.scanerSub = self.create_subscription(LaserScan, f"/pioneer{self.pioneerNo}/scan", self.readLaserScan, 10)

        self.firstOdom = True
        self.initOdom = [0,0,0]

        self.linearSpeed = 0.1
        self.angularSpeed = 0.1

        self.settedAngle = 0.0
        self.settedPosition = 0.0

        plt.ion() 

        self.create_subscription(Odometry, f"/pioneer{self.pioneerNo}/odom", self.callback_odom, 10)
        
        self.veloPublisher = self.create_publisher(Twist, f"/pioneer{self.pioneerNo}/cmd_vel", 10)

        self.filename = "Map.csv"
        
        self.timer = self.create_timer(0.2, self.moveRobot)

    def bayes_update(self, prior, likelihood):
        a = prior * likelihood / (prior * likelihood + (1 - prior) * (1 - likelihood))
        print("Prob: ", a)
        return a

    def callback_odom(self, msg):

        x_curr = msg.pose.pose.position.x
        y_curr = msg.pose.pose.position.y
        theta_curr = yaw_from_quaternion(msg.pose.pose.orientation)

        if self.firstOdom:
            self.firstOdom = False
            self.initOdom = [x_curr,
                            y_curr,
                            theta_curr]
            
        x_curr -= self.initOdom[0]
        y_curr -= self.initOdom[1]

        self.robotPosition[0] = self.robotStartPosition[0] + int(x_curr * self.scalingFactor)
        self.robotPosition[1] = self.robotStartPosition[1] + int(y_curr * self.scalingFactor)
        self.robotOrientation = theta_curr
        
    def printGrid(self):
        self.img.set_data(self.grid)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.show()
        
    def updateGrid(self):
        occupation_factor = 1.1
        free_factor = 0.9

        for i in range(MAP_WIDTH):
            for j in range(MAP_LENGTH):
                if self.scan[i][j] == 1:
                    if self.grid[i][j] * occupation_factor < OCCUPIED_FIELD_PROB:
                        self.grid[i][j] = self.grid[i][j] * occupation_factor
                    else:
                        self.grid[i][j] = OCCUPIED_FIELD_PROB
                elif self.scan[i][j] == 0:
                    if self.grid[i][j] * free_factor > FREE_FIELD_PROB:
                        self.grid[i][j] = self.grid[i][j] * free_factor
                    else:
                        self.grid[i][j] = FREE_FIELD_PROB


    def readLaserScan(self, msg):

        if self.firstOdom:
            return

        scan=list(msg.ranges)
        scan_cartesian = []
        ang = msg.angle_min

        self.scan[self.robotPosition[0]][self.robotPosition[1]] = FREE_FIELD_PROB

        for elem in scan:
            if elem < 6.0:
                scan_cartesian.append(pol2cart(elem*self.scalingFactor , math.degrees(ang + self.robotOrientation)))
            ang += msg.angle_increment

        ang = 0

        for s in scan_cartesian:
            dist_x = int(s[0] + self.robotPosition[0])
            dist_y = int(s[1] + self.robotPosition[1])

            if self.robotPosition[0] != dist_x and \
                self.robotPosition[1] != dist_y:

                coords = breshnamm.bresenham(self.robotPosition[0], self.robotPosition[1], 
                                        dist_x, dist_y)
                for coord in coords:
                    self.scan[coord[0]][coord[1]] = 0

            self.scan[dist_x][dist_y] = 1

        self.updateGrid()
        # self.printGrid()

        self.scan = np.full(shape=(MAP_WIDTH, MAP_LENGTH), fill_value=DEAFULT_FIELD_PROB)
        self.scan[self.robotPosition[0]][self.robotPosition[1]] = FREE_FIELD_PROB

    def saveMap(self):
        with open(self.filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(self.grid)
        print("Map saved succesfully!")

    def loadMap(self):
        with open(self.filename, mode='r') as file:
            reader = csv.reader(file)
            self.grid = [[float(value) for value in row] for row in reader]

    def sendVelo(self, lin, ang):
        msg = Twist()
        vec = Vector3()
        vec.x = float(lin)
        vec.y = 0.0
        vec.z = 0.0
        msg.linear = vec

        vec2 = Vector3()
        vec2.x = 0.0
        vec2.y = 0.0
        vec2.z = float(ang)
        msg.angular = vec2

        self.veloPublisher.publish(msg)

    def moveRobot(self):
        error = 0.1
        velo_ang = 0
        lin_velo = 0

        if abs(self.settedAngle - self.robotOrientation) > error:
            if self.settedAngle > self.robotOrientation:
                velo_ang = 0.2
            else:
                velo_ang = -0.2
        else:
            velo_ang = 0.0

        if abs(self.settedPosition - self.robotPosition[0]) > error:
            if self.settedPosition > self.robotPosition[0]:
                lin_velo = -0.2
            else:
                lin_velo = 0.2
        else:
            lin_velo = 0.0

        self.sendVelo(0, velo_ang)

        print(f"Robot orientation: {self.robotOrientation}, robot pos: {self.robotPosition[0]}")

    def setAngle(self, ang):
        self.settedAngle = (((ang + 3.15) % 6.29 ) - 3.15)

    def rotateRobot(self, ang):
        self.setAngle(self.robotOrientation + ang)

    def moveRobotF(self, dist):
        self.settedPosition = dist

def main(args=None):

    rclpy.init()

    mNode = OccupancyGridMap()

    # mNode.loadMap()
    # mNode.printGrid()
    # mNode.printGrid()
    # while True:
    #     time.sleep(2)
    #     mNode.moveRobot(0, 3.14/4.0)
        # mNode.sendVelo(0.1, 0)
        # time.sleep(1)
        # mNode.sendVelo(0, 0)
        # time.sleep(1)
        # mNode.sendVelo(0, 0.1)
        # time.sleep(1)
        # mNode.sendVelo(0, 0)

    # mNode.settedAngle = ((ang + mNode.robotOrientation + 3.14) % 6.28) - 3.14
    mNode.rotateRobot(1.57)
    # mNode.moveRobotF(1)
    rclpy.spin(mNode)
    mNode.destroy_node()
    rclpy.shutdown()
    


if __name__ == "__main__":
    main(args=None)   



