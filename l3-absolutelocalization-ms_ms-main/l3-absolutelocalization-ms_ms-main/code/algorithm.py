#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from decawave_msgs.msg import Tag #tag_pose
from decawave_msgs.msg import AnchorArray #tag_status
from decawave_msgs.msg import Anchor
import numpy as np
#from zzz.msg import ZZZ #odom


# Zrozumiałem to tak:
# 1. Subscribery po tych trzech topic'ach co są przygotowane, żeby odczytać z nich dane (na tą chwilę nie wiem jakie)
# 2. Nie wiem skąd dostaniemy info o pozycjach beckon'ów (chyba, któryś z topic'ów zwraca coś konretnego - do sprawdzenia)
# 3. Trzeba napisać własny algorytm lokalizujący a następnie porównać go z zaimplementowanymi (tag_pose i odom) oraz naszą wcześniej napisaną odometrią


class Beckon(Node):
    def __init__(self):
        super().__init__("beckon")
        self.sub_tag_pose = self.create_subscription(Tag, "/pioneer2/tag_pose", self.tag_pose_callback, 1)

        self.anchor_x = []
        self.anchor_y = []
        self.anchor_z = []
        self.distance_ = []
        self.counter = 0
        self.pos = []
        self.rob_pos = []
        self.error = []

        self.sub_tag_status = self.create_subscription(AnchorArray, "/pioneer2/tag_status", self.tag_status_callback, 1)
        #self.sub_odom = self.create_subscription(ZZZ, "/pioneer5/odom", self.odom_callback, 10)
        #self.get_logger().info(str(self.sub_tag_status))

        



    def tag_pose_callback(self,msg: Tag):
        self.rob_pos = [msg.x, msg.y]
        #self.get_logger().info(str(msg))


    def tag_status_callback(self,msg: AnchorArray):
        while self.counter < 4:
            self.anchor_x.append(msg.anchors[self.counter].x)
            self.anchor_y.append(msg.anchors[self.counter].y)
            self.anchor_z.append(msg.anchors[self.counter].z + self.counter/1000)
            self.distance_.append(msg.anchors[self.counter].distance)
            self.counter += 1
            #print(self.counter)

        

        #print(self.distance_)
        #print(self.anchor_x,self.anchor_y,self.anchor_z, self.distance)
        self.algorithm()
            

    def algorithm(self):
        # A*q = B

        self.A = np.matrix([[self.anchor_x[1]-self.anchor_x[0], self.anchor_y[1]-self.anchor_y[0], self.anchor_z[1]-self.anchor_z[0]] , 
                          [self.anchor_x[2]-self.anchor_x[0], self.anchor_y[2]-self.anchor_y[0], self.anchor_z[2]-self.anchor_z[0]] , 
                          [self.anchor_x[3]-self.anchor_x[0], self.anchor_y[3]-self.anchor_y[0], self.anchor_z[3]-self.anchor_z[0]]])
        self.B = np.matrix([[(self.distance_[0]**2-self.distance_[1]**2-self.anchor_x[0]**2 + self.anchor_x[1]**2 - self.anchor_y[0]**2 + self.anchor_y[1]**2 - self.anchor_z[0]**2 + self.anchor_z[1]**2)/2] ,
                           [(self.distance_[0]**2-self.distance_[2]**2-self.anchor_x[0]**2 + self.anchor_x[2]**2 - self.anchor_y[0]**2 + self.anchor_y[2]**2 - self.anchor_z[0]**2 + self.anchor_z[2]**2)/2] , 
                           [(self.distance_[0]**2-self.distance_[3]**2-self.anchor_x[0]**2 + self.anchor_x[3]**2 - self.anchor_y[0]**2 + self.anchor_y[3]**2 - self.anchor_z[0]**2 + self.anchor_z[3]**2)/2]])
        #print(self.B)
        #print(self.A)

        self.A = np.matrix([[self.anchor_x[1]-self.anchor_x[0], self.anchor_y[1]-self.anchor_y[0]] , 
                          [self.anchor_x[2]-self.anchor_x[0], self.anchor_y[2]-self.anchor_y[0]]])
        self.B = np.matrix([[(self.distance_[0]**2-self.distance_[1]**2-self.anchor_x[0]**2 + self.anchor_x[1]**2 - self.anchor_y[0]**2 + self.anchor_y[1]**2 )/2] ,
                           [(self.distance_[0]**2-self.distance_[2]**2-self.anchor_x[0]**2 + self.anchor_x[2]**2 - self.anchor_y[0]**2 + self.anchor_y[2]**2 )/2]])
        

        self.pos = np.linalg.solve(self.A,self.B)
        #print(self.pos)
        self.error = [[self.rob_pos[0] - self.pos[0]] , [self.rob_pos[1] - self.pos[1]]]
        print(self.error[0], self.error[1])
        
        
        self.anchor_x.clear()
        self.anchor_y.clear()
        self.anchor_z.clear()
        self.distance_.clear()
        self.counter = 0
        #print()
        #self.beckon[0] = msg.anchors[0]
        #self.beckon_1.y = msg.y
        #self.beckon_1.z = msg.z
        #self.get_logger().info(str(msg.anchors[0].x))


    #def odom_callback(self,msg: ZZZ):
        
        #self.beckon_1.x = msg.x
        #self.beckon_1.y = msg.y
        #self.beckon_1.z = msg.z
        #self.get_logger().info(str(msg))




def main(args=None):
    rclpy.init(args=args)
    node = Beckon()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
  main(args=None)   
