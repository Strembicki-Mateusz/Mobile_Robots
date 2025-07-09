import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

f = np.array([[1,2,3], [4,5,6], [7,8,9]])
plt.imshow(f, interpolation="nearest",cmap='Blues')
plt.colorbar()
plt.show() 
