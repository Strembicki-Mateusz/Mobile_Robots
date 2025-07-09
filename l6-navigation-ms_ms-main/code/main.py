#!/usr/bin/env python
from sensor_msgs.msg import LaserScan
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import math
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import breshnamm 
import csv
import time
import path_finder as a_star

ROOM_WIDTH = 20
ROOM_LENGHT = 20
MAP_WIDTH = 200
MAP_LENGTH = 200
CELL_SIZE = float(MAP_LENGTH)/float(ROOM_LENGHT)
ROBOT_SIZE = 6



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
        self.pioneerNo = 6

        self.flag_rot = 1
        self.scalingFactor = CELL_SIZE
        self.found_path = ()
        self.grid = np.full(shape=(MAP_WIDTH, MAP_LENGTH), fill_value=DEAFULT_FIELD_PROB)
        self.scan = np.full(shape=(MAP_WIDTH, MAP_LENGTH), fill_value=DEAFULT_FIELD_PROB)
        self.path = np.full(shape=(int(MAP_WIDTH / ROBOT_SIZE), int(MAP_LENGTH / ROBOT_SIZE)), fill_value=0.0)

        self.robotStartPosition = [int(MAP_WIDTH/2), int(MAP_LENGTH/2)]
        self.robotPosition = [int(MAP_WIDTH/2), int(MAP_LENGTH/2)]
        self.robotOrientation = 0.0
        self.grid[self.robotPosition[0]][self.robotPosition[1]] = 1 # tu stoi robot na starcie
        self.target_pos = (int(MAP_WIDTH/2) - 200, int(MAP_LENGTH/2) + 80)
        self.start_pos = tuple(self.robotPosition)


        self.fig, self.ax = plt.subplots()
        self.img = self.ax.imshow(self.grid, interpolation="nearest", cmap='Blues', norm=mcolors.Normalize(vmin=0, vmax=1))
        self.colorbar = self.fig.colorbar(self.img)

        self.scanerSub = self.create_subscription(LaserScan, f"/pioneer{self.pioneerNo}/scan", self.readLaserScan, 10)
        self.veloPublisher = self.create_publisher(Twist, f"/pioneer{self.pioneerNo}/cmd_vel", 10)

        self.firstOdom = True
        self.initOdom = [0,0,0]

        self.linearSpeed = 0.1
        self.angularSpeed = 0.1

        self.settedAngle = 0.0
        self.prevPos = 0.0
        self.distToGo = 0.0

        self.moveDone = True

        self.rot_right = 11
        self.rot_left = 12
        self.move = 13

        self.timer = self.create_timer(0.2, self.moveRobot)

        plt.ion() 

        self.create_subscription(Odometry, f"/pioneer{self.pioneerNo}/odom", self.callback_odom, 10)
        self.timer2 = self.create_timer(0.5, self.doSomething)

        self.filename = "Map.csv"
        # timer do zapisania mapy i wylaczenia programu po okreslonym czasie
        # self.timer = self.create_timer(45, self.timer_callback)

    def doSomething(self):
        if self.found_path != None:
            moveToDo = self.JAZDA()
        
            if moveToDo == self.rot_right:
                print("Rotacja prawo!")
            elif moveToDo == self.rot_left:
                print("Rotacja lewo!")
            elif moveToDo == self.move:
                print("Prosto!")
            elif moveToDo == 0:
                print("Brak komendy")
            else:
                print("Bledna komenda")
        else:
            print("Brak sciezki")
        
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
            self.settedAngle = theta_curr
            
        x_curr -= self.initOdom[0]
        y_curr -= self.initOdom[1]

        self.robotPosition[0] = self.robotStartPosition[0] + int(x_curr * self.scalingFactor)
        self.robotPosition[1] = self.robotStartPosition[1] + int(y_curr * self.scalingFactor)
        self.robotOrientation = theta_curr
        
    # def printGrid(self):
    #     self.img.set_data(self.grid)
    #     self.fig.canvas.draw()
    #     self.fig.canvas.flush_events()
    #     plt.show()

    def printGrid(self):
        self.ax.clear()
        self.img = self.ax.imshow(self.path, interpolation="nearest", cmap='Blues', norm=mcolors.Normalize(vmin=0, vmax=1))

        if self.found_path:
            x_coords, y_coords = zip(*self.found_path)
            self.ax.plot(y_coords, x_coords, color='red', linewidth=2)
        
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
        self.printGrid()
        self.scale_map()

        self.scan = np.full(shape=(MAP_WIDTH, MAP_LENGTH), fill_value=DEAFULT_FIELD_PROB)
        self.scan[self.robotPosition[0]][self.robotPosition[1]] = FREE_FIELD_PROB

    def saveMap(self):
        with open(self.filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(self.grid)
        print("Map saved succesfully!")

    def timer_callback(self):
        self.saveMap()
        exit()

    def loadMap(self):
        with open(self.filename, mode='r') as file:
            reader = csv.reader(file)
            self.grid = [[float(value) for value in row] for row in reader]

    def Move(self):
        self.moveRobotF(-1.0 / ROBOT_SIZE)

    def Rotate(self, ang):
        self.rotateRobot(ang)

    def scale_map(self):
        new_width = MAP_WIDTH // ROBOT_SIZE
        new_length = MAP_LENGTH // ROBOT_SIZE
        self.start_pos = (int(new_width/2) , int(new_length/2))
        self.target_pos = (int(new_width/2) + 3, int(new_length/2) - 4)
        self.robotPosPath = (int(self.robotPosition[0]// ROBOT_SIZE) , int(self.robotPosition[1]// ROBOT_SIZE))
        

        for i in range(new_width):
            for j in range(new_length):
                for x in range(i * ROBOT_SIZE, (i + 1) * ROBOT_SIZE):
                    for y in range(j * ROBOT_SIZE, (j + 1) * ROBOT_SIZE):
                        if x < MAP_WIDTH and y < MAP_LENGTH:  # Zapewniamy, że nie wyjdziemy poza granice
                            if self.grid[x][y] == FREE_FIELD_PROB:
                                self.path[i][j] = 0
                            elif self.grid[x][y] == DEAFULT_FIELD_PROB:
                                self.path[i][j] = 0
                            else:
                                self.path[i][j] = 1

        
        self.found_path = a_star.dynamic_path_correction(self.path, self.robotPosPath, self.target_pos)
        #print(self.robotPosPath)
        #print(self.target_pos)
        #print(tuple(np.abs(np.subtract(self.found_path[0], self.found_path[1]))))
        # print(self.found_path)
        # if self.found_path != None:
        #     self.JAZDA()
        # else:
        #     print("Dojechalem")

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
        velo_ang = 0.0
        lin_velo = 0.0

        if abs(self.settedAngle - self.robotOrientation) > error:
            if self.settedAngle > self.robotOrientation:
                velo_ang = 0.2
            else:
                velo_ang = -0.2
        else:
            velo_ang = 0.0

        robotPos = np.sqrt(self.robotPosition[0]**2 + self.robotPosition[1]**2)

        if self.distToGo > 0.0:
            if self.prevPos + self.distToGo > robotPos:
                lin_velo = -0.2
            else:
                lin_velo = 0.0
                self.distToGo = 0.0

        elif self.distToGo < 0.0:
            if self.prevPos + self.distToGo < robotPos:
                lin_velo = 0.2
            else:
                lin_velo = 0.0
                self.distToGo = 0.0

        else:
            self.prevPos = robotPos
        
        self.moveDone = lin_velo == 0.0 and velo_ang == 0.0

        self.sendVelo(lin_velo, velo_ang)

        # print(f"Moved dist: {self.prevPos}, distToGo: {self.distToGo}, pos: {np.sqrt(self.robotPosition[0]**2 + self.robotPosition[1]**2)}")

    def setAngle(self, ang):
        self.settedAngle = (((ang + 3.15) % 6.29 ) - 3.15)

    def rotateRobot(self, ang):
        self.moveDone = False
        self.setAngle(self.robotOrientation + ang)

    def moveRobotF(self, dist):
        self.moveDone = False
        self.distToGo = dist 
        self.prevPos = np.sqrt(self.robotPosition[0]**2 + self.robotPosition[1]**2)


    def JAZDA(self):
        direction_up = 0
        direction_right = 1
        direction_down = 2
        direction_left = 3
        last_direction = 0 #orientacja wcześniejsza
        current_direction = 0 #orientacja obecna

        if (self.found_path[0][0] - self.found_path[0][1]) == 1 and (self.found_path[0][1] - self.found_path[1][1]) == 0:
            if last_direction == direction_right:
                # self.Move()
                return self.move
            else:
                if current_direction == direction_up:
                    # self.Rotate(np.deg2rad(90))
                    return self.rot_right
                elif current_direction == direction_right:
                    # self.Move()
                    return self.move
                elif current_direction == direction_down:
                    # self.Rotate(np.deg2rad(-90))
                    return self.rot_left
                elif current_direction == direction_left:
                    print()#self.Rotate(np.deg2rad(180))

            last_direction = current_direction
            current_direction = direction_right
        




        elif (self.found_path[0][0] - self.found_path[0][1]) == 0 and (self.found_path[0][1] - self.found_path[1][1]) == 1:
            if last_direction == direction_up:
                # self.Move()
                return self.move
            else:
                if current_direction == direction_up:
                    # self.Move()
                    return self.move
                elif current_direction == direction_right:
                    # self.Rotate(np.deg2rad(-90))
                    return self.rot_left
                elif current_direction == direction_down:
                    print()#self.Rotate(np.deg2rad(180))
                elif current_direction == direction_left:
                    # self.Rotate(np.deg2rad(90))
                    return self.rot_right


            last_direction = current_direction
            current_direction = direction_up

        elif (self.found_path[0][0] - self.found_path[0][1]) == -1 and (self.found_path[0][1] - self.found_path[1][1]) == 0:
            if last_direction == direction_left:
                # self.Move()
                return self.move
            else:
                if current_direction == direction_up:
                    # self.Rotate(np.deg2rad(-90))
                    return self.rot_left
                elif current_direction == direction_right:
                    print()#self.Rotate(np.deg2rad(180))
                elif current_direction == direction_down:
                    # self.Rotate(np.deg2rad(90))
                    return self.rot_right
                elif current_direction == direction_left:
                    # self.Move()
                    return self.move

            last_direction = current_direction
            current_direction = direction_left
            # 0 -1
        elif (self.found_path[0][0] - self.found_path[0][1]) == 0 and (self.found_path[0][1] - self.found_path[1][1]) == -1:
            if last_direction == direction_down:
                # self.Move()
                return self.move
            else:
                if current_direction == direction_up:
                    print()#self.Rotate(np.deg2rad(180))
                elif current_direction == direction_right:
                    # self.Rotate(np.deg2rad(90))
                    return self.rot_right
                elif current_direction == direction_down:
                    # self.Move()
                    return self.move
                elif current_direction == direction_left:
                    # self.Rotate(np.deg2rad(-90))
                    return self.rot_left

            last_direction = current_direction
            current_direction = direction_down
                
        #print(f"{current_direction}")
        return 0

 
    
    

def main(args=None):

    rclpy.init()

    mNode = OccupancyGridMap()

    # odkomentuj, zeby zaladowac mape

    # mNode.loadMap()
    # mNode.printGrid()
    # mNode.printGrid()
    # while True:
    #     time.sleep(100)

    rclpy.spin(mNode)
    mNode.destroy_node()
    rclpy.shutdown()
    


if __name__ == "__main__":
    main(args=None)   



