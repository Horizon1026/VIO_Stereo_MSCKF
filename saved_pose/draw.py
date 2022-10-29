import numpy as np
from matplotlib import pyplot as plt

states = np.loadtxt("estimated_poses.txt")
timeStamp = states[:, 0]
atti = states[:, 1:5]
pos = states[:, 5:8]
vel = states[:, 8:11]

fig = plt.figure('pos')
ax = plt.axes(projection='3d')
ax.plot3D(pos[:, 0], pos[:, 1], pos[:, 2], '-b')

# fig = plt.figure('atti')
# plt.plot(atti[:, 0], atti_truth[:, 0], '-r')
# plt.plot(atti[:, 0], atti_truth[:, 1], '-b')
# plt.plot(atti[:, 0], atti_truth[:, 2], '-k')
# plt.plot(atti[:, 0], atti_truth_q1[:, 0], '-r')
# plt.plot(atti[:, 0], atti_truth_q1[:, 1], '-b')
# plt.plot(atti[:, 0], atti_truth_q1[:, 2], '-k')
plt.show()