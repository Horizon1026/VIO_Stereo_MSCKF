import numpy as np
from matplotlib import pyplot as plt

states = np.loadtxt("estimated_poses.txt")
timeStamp = states[:, 0]
quat = states[:, 1:5]
pos = states[:, 5:8]
vel = states[:, 8:11]
ba = states[:, 11:14]
bg = states[:, 14:17]


# translate quaternion to pitch roll yaw
# todo


fig = plt.figure('trajactory')
ax = plt.axes(projection='3d')
ax.plot3D(pos[:, 0], pos[:, 1], pos[:, 2], color = 'blue')


fig, ax = plt.subplots(nrows = 5, ncols = 1)

ax[0].set_title('rotation')
ax[0].grid()
ax[0].plot(timeStamp, quat[:, 0], color = 'red')
ax[0].plot(timeStamp, quat[:, 1], color = 'green')
ax[0].plot(timeStamp, quat[:, 2], color = 'blue')
ax[0].plot(timeStamp, quat[:, 3], color = 'black')
ax[0].legend(['w', 'x', 'y', 'z'])

ax[1].set_title('position')
ax[1].grid()
ax[1].plot(timeStamp, pos[:, 0], color = 'red')
ax[1].plot(timeStamp, pos[:, 1], color = 'green')
ax[1].plot(timeStamp, pos[:, 2], color = 'blue')
ax[1].legend(['x', 'y', 'z'])

ax[2].set_title('velocity')
ax[2].grid()
ax[2].plot(timeStamp, vel[:, 0], color = 'red')
ax[2].plot(timeStamp, vel[:, 1], color = 'green')
ax[2].plot(timeStamp, vel[:, 2], color = 'blue')
ax[2].legend(['x', 'y', 'z'])

ax[3].set_title('bias accel')
ax[3].grid()
ax[3].plot(timeStamp, ba[:, 0], color = 'red')
ax[3].plot(timeStamp, ba[:, 1], color = 'green')
ax[3].plot(timeStamp, ba[:, 2], color = 'blue')
ax[3].legend(['x', 'y', 'z'])

ax[4].set_title('bias gyro')
ax[4].grid()
ax[4].plot(timeStamp, bg[:, 0], color = 'red')
ax[4].plot(timeStamp, bg[:, 1], color = 'green')
ax[4].plot(timeStamp, bg[:, 2], color = 'blue')
ax[4].legend(['x', 'y', 'z'])

plt.show()