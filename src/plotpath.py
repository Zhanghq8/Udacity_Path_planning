#!/usr/bin/env python

import matplotlib.pyplot as plt

import numpy as np

path = np.loadtxt(open("../map/finegrained_map.csv","rb"),delimiter=" ",skiprows=0)
path_left = np.loadtxt(open("../map/leftlane_map.csv","rb"),delimiter=" ",skiprows=0)
path_mid = np.loadtxt(open("../map/midlane_map.csv","rb"),delimiter=" ",skiprows=0)
path_right = np.loadtxt(open("../map/rightlane_map.csv","rb"),delimiter=" ",skiprows=0)
path_x = np.loadtxt(open("../xpath.txt","rb"),delimiter=" ",skiprows=0)
path_y = np.loadtxt(open("../ypath.txt","rb"),delimiter=" ",skiprows=0)

x_path = path[:,0].tolist()
y_path = path[:,1].tolist()

x_path_left = path_left[:, 0].tolist()
y_path_left = path_left[:, 1].tolist()

x_path_mid = path_mid[:, 0].tolist()
y_path_mid = path_mid[:, 1].tolist()

x_path_right = path_right[:, 0].tolist()
y_path_right = path_right[:, 1].tolist()

x_path_wp = path_x.tolist()
y_path_wp = path_y.tolist()

wpa = [908.48, 909.48, 937.418, 967.228, 996.388]
wpb = [1128.67, 1128.67, 1135.19, 1138.72, 1145.69]

plt.figure('Path figure')
ax = plt.gca()

# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.grid()
# ax.set_xlim(left=900, right=1200)
# ax.set_ylim(bottom=1000, top=1200)
# ax.legend()
# ax.plot(x_path, y_path, color='r', marker='.', markersize=0.1, linewidth=0.1, alpha=1.0, label='Path')
# ax.plot(x_path_left, y_path_left, color='b', marker='.', markersize=0.1, linewidth=0.1, alpha=1.0, label='Path Left')
# ax.plot(x_path_mid, y_path_mid, color='g', marker='.', markersize=0.1, linewidth=0.1, alpha=1.0, label='Path Mid')
# ax.plot(x_path_right, y_path_right, color='c', marker='.', markersize=0.1, linewidth=0.1, alpha=1.0, label='Path Right')
# ax.plot(wpa, wpb, color='k', marker='*', markersize=0.3, linewidth=0.1, alpha=1.0, label='Path WP')


while (1):

	for i in range(len(x_path_wp)):
		# print (i)
		wp_x = []
		wp_y = []
		for j in range(len(x_path_wp[0])):
			wp_x.append(x_path_wp[i][j])
			wp_y.append(y_path_wp[i][j])
		ax.clear()

		ax.set_xlabel('x')
		ax.set_ylabel('y')
		ax.grid()
		ax.set_xlim(left=wp_x[0]-100, right=wp_x[0]+300)
		ax.set_ylim(bottom=wp_y[0]-50, top=wp_y[0]+100)
		ax.legend()
		ax.plot(x_path, y_path, color='r', marker='.', markersize=0.1, linewidth=0.1, alpha=1.0, label='Path')
		ax.plot(x_path_left, y_path_left, color='b', marker='.', markersize=0.1, linewidth=0.1, alpha=1.0, label='Path Left')
		ax.plot(x_path_mid, y_path_mid, color='g', marker='.', markersize=0.1, linewidth=0.1, alpha=1.0, label='Path Mid')
		ax.plot(x_path_right, y_path_right, color='c', marker='.', markersize=0.1, linewidth=0.1, alpha=1.0, label='Path Right')
		# print wp_x
		# print wp_y
		ax.plot(wp_x, wp_y, color='k', marker='.', markersize=0.5, linewidth=0.4, alpha=1.0, label='Path way point')
		# ax.set_xlim(left=-4, right=4)
		plt.draw()
		plt.pause(0.01)
plt.draw()
plt.show()





