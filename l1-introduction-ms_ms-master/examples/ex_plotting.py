import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

from time import sleep

loop_count=25  # drawing loop iterations
beam_half_angle=7.5 # half of angular beam width

# 
# A function to calculate Cartesian coordinates to polar
#  result: a tuple (rho,phi)
#  rho - radius, phi - angle in degrees
def cart2pol(x, y):
    #TODO: calculations
    rho = math.sqrt(x**2 + y**2)
    phi = math.degrees(math.asin(y/rho))
    return(rho, phi)

# A function to transform polar  coordinates to Cartesian
# input angle in degrees
# returns a tuple (x,y)
def pol2cart(rho, phi):
    #TODO: calculations
    x = rho*math.sin(math.radians(phi))
    y = rho*math.cos(math.radians(phi))
    return(x, y)

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


plt.figure()
ax = plt.gca()
ax.set_aspect('equal')
ax.set_xlim([-6,6])
ax.set_ylim([-6,6])
plt.ion()
plt.show()

for i in range(loop_count):
    skan=[[1,0],[1,1],[1.5,0.8]]
    ax.cla()
    plotwedges(ax,skan)  
    plotarrows(ax,skan)
    ax.set_xlim([-6,6])
    ax.set_ylim([-6,6])
    plt.draw()
    plt.pause(0.0001)
    sleep(0.2)
