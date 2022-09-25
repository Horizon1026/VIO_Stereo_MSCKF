import numpy as np
from matplotlib import pyplot as plt

atti = np.loadtxt("atti.txt")
atti_truth = np.loadtxt("atti_truth.txt")
atti_truth_v0 = np.loadtxt("atti_truth_init_vel_tobe_0.txt")
atti_truth_q1 = np.loadtxt("atti_truth_init_atti_tobe_1.txt")
atti_truth = atti_truth[0:len(atti), :]
atti_truth_v0 = atti_truth_v0[0:len(atti), :]
atti_truth_q1 = atti_truth_q1[0:len(atti), :]

fig = plt.figure('pos')
ax = plt.axes(projection='3d')
ax.plot3D(atti_truth[:, 3], atti_truth[:, 4], atti_truth[:, 5], '-b')
ax.plot3D(atti_truth_v0[:, 3], atti_truth_v0[:, 4], atti_truth_v0[:, 5], '-r')
ax.plot3D(atti_truth_q1[:, 3], atti_truth_q1[:, 4], atti_truth_q1[:, 5], '-g')

fig = plt.figure('atti')
plt.plot(atti[:, 0], atti_truth[:, 0], '-r')
plt.plot(atti[:, 0], atti_truth[:, 1], '-b')
plt.plot(atti[:, 0], atti_truth[:, 2], '-k')
plt.plot(atti[:, 0], atti_truth_q1[:, 0], '-r')
plt.plot(atti[:, 0], atti_truth_q1[:, 1], '-b')
plt.plot(atti[:, 0], atti_truth_q1[:, 2], '-k')
plt.show()