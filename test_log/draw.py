import numpy as np
from matplotlib import projections, pyplot as plt

pos = np.loadtxt("pos.txt");

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(pos[:, 0], pos[:, 1], pos[:, 2])
plt.show()