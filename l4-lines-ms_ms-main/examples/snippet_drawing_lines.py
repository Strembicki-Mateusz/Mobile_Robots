import numpy as np
import pylab as pl
from matplotlib import collections as mc

# Define first set of line segments
lines = [[(0, 1), (1, 1)], [(2, 3), (3, 3)], [(1, 2), (1, 3)]]
colors = np.array([(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)])  # Red, Green, Blue colors with full opacity

# Create a LineCollection for the first set of lines
lc = mc.LineCollection(lines, colors=colors, linewidths=2)

# Define second set of line segments
lines2 = [[(0.1, 1.1), (1.1, 1.1)], [(2.1, 3.1), (3.1, 3.1)], [(1.1, 2.1), (1.1, 3.1)]]

# Create a LineCollection for the second set of lines with black color
lc2 = mc.LineCollection(lines2, colors=[0, 0, 0], linewidths=1)

# Create the plot
fig, ax = pl.subplots()
ax.add_collection(lc)    # Add the first LineCollection
ax.add_collection(lc2)   # Add the second LineCollection

# Adjust plot scaling and margins
ax.autoscale()
ax.margins(0.1)

# Display the plot
pl.show()
