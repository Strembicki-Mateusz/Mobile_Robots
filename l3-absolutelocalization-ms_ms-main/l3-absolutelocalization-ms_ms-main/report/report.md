# Report 

### Authors

- Mateusz Strembicki
- Marcel Skrok

## Results
The following algorithm was develop, where:
- A is a matrix describing distance beetwen anchors
- B is a vector describing distance to the robot
- q is a vector of searching parameters (position of the robot) q = [x,y,z]^T
```
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

```

There was tested two aproaches.
- first using all four anchors and read data from them. These solution has some advantages. Firstly, using these one it can be obtained the third parameter 'z' which don't need to be calculatoed, because of knowgledge about the enviroment in which robot moving. Secondly, it can gives better solution for solving task about localization robot using anchors, because there are used all of the beacons and we should depend on all of them. Worth to mention is also that if one of anchor does not work correctly and not always give us info about position the algorithm is going to failed the solution and it doesnt work.
  ```
  self.A = np.matrix([[self.anchor_x[1]-self.anchor_x[0], self.anchor_y[1]-self.anchor_y[0], self.anchor_z[1]-self.anchor_z[0]] , 
                          [self.anchor_x[2]-self.anchor_x[0], self.anchor_y[2]-self.anchor_y[0], self.anchor_z[2]-self.anchor_z[0]] , 
                          [self.anchor_x[3]-self.anchor_x[0], self.anchor_y[3]-self.anchor_y[0], self.anchor_z[3]-self.anchor_z[0]]])
        self.B = np.matrix([[(self.distance_[0]**2-self.distance_[1]**2-self.anchor_x[0]**2 + self.anchor_x[1]**2 - self.anchor_y[0]**2 + self.anchor_y[1]**2 - self.anchor_z[0]**2 + self.anchor_z[1]**2)/2] ,
                           [(self.distance_[0]**2-self.distance_[2]**2-self.anchor_x[0]**2 + self.anchor_x[2]**2 - self.anchor_y[0]**2 + self.anchor_y[2]**2 - self.anchor_z[0]**2 + self.anchor_z[2]**2)/2] , 
                           [(self.distance_[0]**2-self.distance_[3]**2-self.anchor_x[0]**2 + self.anchor_x[3]**2 - self.anchor_y[0]**2 + self.anchor_y[3]**2 - self.anchor_z[0]**2 + self.anchor_z[3]**2)/2]])
  ```
- second aproach is simillary to the first, but we depend only on (chosen) three anchors and we let go of the control of the third parameter which is the z-coordinate. This aproach is simpler to solve because it is 2-dimensional problem. Other of the positive aspect of this aproach is that if one anchors does not send any information, it can be replaced by the other which is unused in main algorithm, so this system is easly reconfigurable.
```
self.A = np.matrix([[self.anchor_x[1]-self.anchor_x[0], self.anchor_y[1]-self.anchor_y[0]] , 
                          [self.anchor_x[2]-self.anchor_x[0], self.anchor_y[2]-self.anchor_y[0]]])
        self.B = np.matrix([[(self.distance_[0]**2-self.distance_[1]**2-self.anchor_x[0]**2 + self.anchor_x[1]**2 - self.anchor_y[0]**2 + self.anchor_y[1]**2 )/2] ,
                           [(self.distance_[0]**2-self.distance_[2]**2-self.anchor_x[0]**2 + self.anchor_x[2]**2 - self.anchor_y[0]**2 + self.anchor_y[2]**2 )/2]])
        
```

## Conclusions
- Localization task is neccesarry to implement in mobile robot systems. Anchors are one of the simple aproach and give some possibilities to use. It can work as a group and robot can choose to which one it should localize. The minus is that every anchors should have well defined inside coordinates to determine robot position in next step. This aproach is also good to implement when we should now where robot is in sense of rooms, floor, etc. or only in one room to determine the position of the robot. 

- The develop algorithm was compared with the inside algorithm of the robot (/tag_pose topic) and the error was very small. The difference beetwen these two algorithm was about 0,02 - 0,04 [m] so it is quite good because the error of anchors was 50 cm.  

