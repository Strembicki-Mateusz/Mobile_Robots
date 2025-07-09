import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import csv
import math
import sys
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import time

class mOdom(Node):

  def __init__(self):
    self.leftWheelPos = 0
    self.rightWheelPos = 0
    self.leftWheelPrevVal = 0
    self.rightWheelPrevVal = 0
    self.leftWheelPrevPos = 0
    self.rightWheelPrevPos = 0
    self.leftWheelZeroPos = 0
    self.rightWheelZeroPos = 0
    self.isInit = False
    self.positionsLeft = []
    self.positionsRight = []
    self.y_constant = 0.5

    self.x_pos = 0
    self.y_pos = 0
    self.theta = 0
    self.x_pos_velo = 0
    self.y_pos_velo = 0
    self.theta_velo = 0
    self.x_positions = []
    self.y_positions = []
    self.x_positions_velo = []
    self.y_positions_velo = []

    self.odom_x = []
    self.odom_y = []
    self.odom_x_initVal = 0
    self.odom_y_initVal = 0
    self.odom_theta = []
    self.isOdomInit = False

    self.prevTime = 0

    self.ticks = 125.1
    self.distanceBetweenWheels = 330
    self.wheeldiameter = 195

    super().__init__('odomReader')
    # self.create_subscription(JointState, f"/bagjoint_states", self.callback_jointStates, 10)
    # self.create_subscription(Odometry, f"/bagodom", self.callback_odom, 10)
    self.create_subscription(JointState, f"/pioneer5/joint_states", self.callback_jointStates, 10)
    self.create_subscription(Odometry, f"/pioneer5/odom", self.callback_odom, 10)

    self.y_points = []
    self.x_points = []

    plt.ion()
    self.fig, (self.ax, self.bx) = plt.subplots(1, 2, figsize=(12, 5)) 
    self.ax.set_xlabel("Sample")
    self.ax.set_ylabel("Position")
    self.ax.set_title("Encoder data")
    self.ax.scatter(self.y_points, self.positionsLeft, color='blue', label='left wheel')
    self.ax.scatter(self.y_points, self.positionsRight, color='red', label='right wheel')
    self.ax.legend(loc='upper right')

    self.bx.set_xlabel("X pos")
    self.bx.set_ylabel("Y pos")
    self.bx.set_title("Robot position")
    # self.bx.set_ylim(-300, 300)
    self.bx.scatter(self.x_positions, self.y_positions, color='green', label='encoder position data')
    self.bx.scatter(self.x_positions_velo, self.y_positions_velo, color='brown', label='encoder velocities')
    self.bx.scatter(self.odom_x, self.odom_y, color='black', label='odom data')
    self.bx.legend(loc='upper right')
    

   
  def csv_reader(self, filename):
    pos_l = []
    pos_r = []
    with open(filename) as csvfile:
      reader = csv.DictReader(csvfile, delimiter=';', quotechar='|')
      for row in reader:        
        pos_l.append(float(row['pos_l']))
        pos_r.append(float(row['pos_r[ticks]']))

    return [pos_l,pos_r]

  def calculate_way(self, pos_l,pos_r):

    for d in range(0, len(pos_l)):
      pos_l[d] = pos_l[d] / self.ticks
      pos_r[d] = pos_r[d] / self.ticks

    return [pos_l, pos_r]

  def calculate_way_s(self, pos_l,pos_r):
    return [pos_l/self.ticks, pos_r/self.ticks]


  def yaw_from_quaternion(self,quaternion):
      """
      Converts quaternion (w in last place) to yaw angle
      quaternion = [x, y, z, w]

      modified from
      https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
      """
      x = quaternion.x
      y = quaternion.y
      z = quaternion.z
      w = quaternion.w

      siny_cosp = 2 * (w * z + x * y)
      cosy_cosp = 1 - 2 * (y * y + z * z)
      yaw = np.arctan2(siny_cosp, cosy_cosp)
      return yaw

  def callback_odom(self, msg):

    pos=msg.pose.pose.position

    if not self.isOdomInit:
      self.isOdomInit = True
      self.odom_x_initVal = pos.x
      self.odom_y_initVal = pos.y
      self.theta += self.yaw_from_quaternion(msg.pose.pose.orientation)
      self.theta_velo += self.yaw_from_quaternion(msg.pose.pose.orientation)

    self.odom_x.append((pos.x - self.odom_x_initVal)*1000.0)
    self.odom_y.append((pos.y - self.odom_y_initVal)*1000.0)
    self.odom_theta.append(self.yaw_from_quaternion(msg.pose.pose.orientation))
    self.bx.scatter(self.odom_x, self.odom_y, color='black', label='odom data')
    # print(f"({pos.x},{pos.y},{self.yaw_from_quaternion(msg.pose.pose.orientation)})")
    

  def callback_jointStates(self, msg):
    
    #######################################################################
    # POSTIONS PART

    readValLeft = msg.position[0]/self.ticks
    readValRight = msg.position[1]/self.ticks

    if not self.isInit:
      self.leftWheelZeroPos = readValLeft
      self.rightWheelZeroPos = readValRight
      readValLeft = readValRight = 0
      self.prevTime = msg.header.stamp.sec + msg.header.stamp.nanosec*(10**-9) - 0.1
      self.isInit = True
    
    #check the encoder overrating
    if self.leftWheelPrevVal * readValLeft >= -10000:
      self.leftWheelPos += (readValLeft - self.leftWheelPrevVal)
    else:
      diff = 2*255.0 - abs(self.leftWheelPrevVal) - abs(readValLeft)
      if self.leftWheelPrevVal < 0:
        self.leftWheelPos -= diff
      else:
        self.leftWheelPos += diff

    if self.rightWheelPrevVal * readValRight >= -10000:
      self.rightWheelPos += (readValRight - self.rightWheelPrevVal)
    else:
      diff = 2*255.0 - abs(self.rightWheelPrevVal) - abs(readValRight)
      if self.rightWheelPrevVal < 0:
        self.rightWheelPos -= diff
      else:
        self.rightWheelPos += diff

        
    self.positionsLeft.append(self.leftWheelPos)
    self.positionsRight.append(self.rightWheelPos)

    if len(self.y_points) == 0:
      self.y_points.append(0)
    else:
      self.y_points.append(self.y_points[-1]+1)

    
    lDist = self.leftWheelPos - self.leftWheelPrevPos
    rDist = self.rightWheelPos - self.rightWheelPrevPos
    dAvg = (lDist + rDist)/2

    self.theta = self.theta + (rDist - lDist)/self.distanceBetweenWheels

    self.x_pos = self.x_pos + dAvg * math.cos(self.theta)
    self.y_pos = self.y_pos + dAvg * math.sin(self.theta)

    self.x_positions.append(self.x_pos)
    self.y_positions.append(self.y_pos)

    self.leftWheelPrevVal = readValLeft
    self.rightWheelPrevVal = readValRight
    self.leftWheelPrevPos = self.leftWheelPos
    self.rightWheelPrevPos = self.rightWheelPos

    ##############################################################################
    # Velocities part

    readVeloLeft = msg.velocity[0]
    readVeloRight = msg.velocity[1]

    currTime = msg.header.stamp.sec + msg.header.stamp.nanosec*(10**-9)

    timeInterval = currTime - self.prevTime
    self.prevTime = currTime

    
    lDist_velo = readVeloLeft*timeInterval
    rDist_velo = readVeloRight*timeInterval
    dAvg = (lDist_velo + rDist_velo)/2

    self.theta_velo = self.theta_velo + (rDist_velo - lDist_velo)/self.distanceBetweenWheels

    self.x_pos_velo = self.x_pos_velo + dAvg * math.cos(self.theta_velo)
    self.y_pos_velo = self.y_pos_velo + dAvg * math.sin(self.theta_velo)

    self.x_positions_velo.append(self.x_pos_velo)
    self.y_positions_velo.append(self.y_pos_velo)


    ##############################################################################
    # Plotting
    self.ax.scatter(self.y_points, self.positionsLeft, color='blue', label='left wheel')
    self.ax.scatter(self.y_points, self.positionsRight, color='red', label='right wheel')
    
    self.bx.scatter(self.x_positions, self.y_positions, color='green', label='encoder pulses')
    self.bx.scatter(self.x_positions_velo, self.y_positions_velo, color='brown', label='encoder velocities')
    plt.draw()
    plt.pause(0.0001)

    print(f"Left dist: {lDist}, left dist velo: {lDist_velo}, right dist: {rDist}, right dist velo: {rDist_velo}")
    # print(f"X: {self.x_pos}, Y: {self.y_pos}, Theta: {self.theta}")

  
def main(args=None):

    rclpy.init()

    mNode = mOdom()
    rclpy.spin(mNode)

    mNode.destroy_node()
    rclpy.shutdown()
    


if __name__ == "__main__":
  main(args=None)   



   
