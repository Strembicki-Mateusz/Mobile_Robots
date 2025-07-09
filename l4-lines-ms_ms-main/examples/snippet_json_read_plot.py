### Scan Processing: Reading JSON File and Plotting

# Import required libraries
import json
import matplotlib.pyplot as plt
import numpy as np

# Reading data from the JSON file
#json_data = open('line_detection_1.json')
#data = json.load(json_data)

def open_json_file(str: str):
  with open(str) as json_data:
    data = json.load(json_data)
  return data

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)
  
# Plotting in polar coordinates

data = open_json_file('./json/line_detection_1.json')
data = open_json_file('./json/line_detection_2.json')
#data = open_json_file('./json/line_localization_1.json')
#distance_list = [x["scan"] for x in data if (str(x) != 'nan') and (str(x) != 'inf')]
#pose_list = [x["pose"] for x in data]
distance_list_tmp = []
distance_list = []
pose_list = []
for x in data:
  pose_list = x["pose"]
  distance_list_tmp = x["scan"]
  
  #(str(distance_list_tmp[x]) != 'nan') and (str(distance_list_tmp[x]) != 'inf')
for x in range(0,len(distance_list_tmp)):
  if (str(distance_list_tmp[x]) != 'nan'):
    print(distance_list_tmp[x])
    distance_list.append(distance_list_tmp[x])
  
#distance_list.remove('inf')

print(distance_list)


#print(distance_list)
x = np.arange(0, len(distance_list))                                    # Note: value for real sensor will be different
theta = (np.pi / len(distance_list)) * (x - len(distance_list)/2)            # Angle in radians

# Reading first dataset (index 0)
scan_data = distance_list

# Create polar plot
fig1 = plt.figure()
ax1 = fig1.add_axes([0.1, 0.1, 0.8, 0.8], polar=True)
line, = ax1.plot(theta, scan_data, lw=2.5)
ax1.set_ylim(0, 2)  # Plot range
plt.show()

# TODO: Modify the code to use Cartesian coordinates
cartezian = []    
for i in range(0,len(distance_list)):
  cartezian.append(pol2cart(distance_list[i],theta[i]))
#print(cartezian)


x_val = []
y_val = []
for i in range(0,len(distance_list)):
  x_val.append(cartezian[i][0])
  y_val.append(cartezian[i][1])


fig2 = plt.figure()
ax2 = fig2.add_axes([0.1, 0.1, 0.8, 0.8], polar=False)
line, = ax2.plot(y_val, x_val, lw=2.5)
ax2.set_ylim(0, 5)  # Plot range
plt.show()
